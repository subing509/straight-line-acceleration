#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Straight tracker with explicit original-protocol test and RES diagnostics."""
from __future__ import print_function
import os
import sys
import time
import math
import json
import threading
from ebs_core import CourseProgress, require_shadow_only
from ebs_output import validate_output_request, exclusive_client_class

# rospkg finds source packages in devel; roslib also resolves install-space code.
import rospkg
import roslib.packages
package = rospkg.RosPack().get_path('acceleration_event')
source = os.path.join(package, 'src')
if not os.path.isfile(os.path.join(source, 'genzong_realtime.py')):
    candidates = roslib.packages.find_node('acceleration_event', 'genzong_realtime.py') or []
    if not candidates:
        raise RuntimeError('acceleration_event tracker and helper modules not found')
    source = os.path.dirname(candidates[0])
sys.path.insert(0, source)
import genzong_realtime as legacy
import rospy
from std_msgs.msg import String

class EBSShadowController(legacy.PathTrackingController):
    def __init__(self):
        self.course = None
        self.ebs_pub = None
        self.ebs_log = None
        self.journal_lock = threading.Lock()
        self.progress_fresh = False
        self.last_conflict_check = 0.0
        self.other_command_publishers = []
        legacy.PathTrackingController.__init__(self)
        self.ebs_pub = rospy.Publisher('/ebs_test/status', String, queue_size=10)
        folder = os.path.join(rospy.get_param('~log_dir', '/tmp/ebs_test_logs'),
                              'ebs_%d_%d' % (int(time.time()*1000), os.getpid()))
        if not os.path.isdir(folder): os.makedirs(folder)
        self.ebs_log = open(os.path.join(folder, 'status.jsonl'), 'a')

    def configure_outputs(self):
        # Runs before the base constructor can create CAN or FSSIM outputs.
        self.output_mode = rospy.get_param('~output_mode', 'shadow_only')
        self.output_profile = rospy.get_param('~profile', 'baseline_shadow')
        self.can_client = None
        self.can_client_ok = False
        self.fssim_available = False
        self.configured_output_enabled = self.enable_vehicle_output
        self.last_can_retry_time = time.time()
        self.course = CourseProgress(rospy.get_param('~acceleration_distance_m', 25.0),
                                     rospy.get_param('~braking_distance_m', 10.0))
        try:
            validate_output_request(self.enable_vehicle_output,
                                    self.output_mode, self.output_profile)
            if self.enable_vehicle_output:
                self.check_command_publishers()
                if self.other_command_publishers:
                    raise RuntimeError('Other command publishers: %s' % self.other_command_publishers)
                client = exclusive_client_class(legacy.CANUDPClient)
                self.can_client = client(rospy.get_param('~can_dest_ip', '192.168.0.7'),
                                         rospy.get_param('~can_dest_port', 20005),
                                         rospy.get_param('~can_self_port', 11312))
                self.can_client_ok = self.can_client.init()
        except Exception:
            if self.can_client is not None: self.can_client.close()
            self.logger.close()
            raise

    def check_command_publishers(self):
        self.last_conflict_check = time.time()
        try:
            result = rospy.get_master().getSystemState()
            if result[0] != 1: raise RuntimeError('master query failed')
            pubs = dict(result[2][0])
            own = rospy.get_name()
            self.other_command_publishers = sorted(set(
                n for topic in ('/control/steering', '/fssim/cmd')
                for n in pubs.get(topic, []) if n != own))
        except Exception:
            self.other_command_publishers = ['master_query_unavailable']

    def refresh_output_readiness(self):
        # No delayed retry or runtime parameter can silently open a socket.
        if self.enable_vehicle_output != self.configured_output_enabled:
            raise RuntimeError('Output mode changes require a node restart')
        if time.time()-self.last_conflict_check > 1.0:
            self.check_command_publishers()
        if self.enable_vehicle_output and self.other_command_publishers:
            self.run_guard.stop('command_channel_conflict')

    def check_start_condition(self):
        if self.enable_vehicle_output and self.other_command_publishers:
            return
        return legacy.PathTrackingController.check_start_condition(self)

    def fssim_pose_callback(self, msg):
        # This EBS entry evaluates the real GPS/IMU chain, not simulation state.
        return

    def update_finish_distance(self):
        # 25/35/75m are not hardware RES events or software finish triggers.
        self.progress_fresh = False
        if not self.is_started or not self.pose_received or not self.run_guard.running or self.run_guard.stop_reason is not None:
            return
        sample = self.run_guard.samples.get('pose')
        age = time.time()-sample[0]+sample[1] if sample else float('inf')
        if not sample or time.time() < sample[0] or age > self.run_guard.limits['pose']:
            return
        axis = self.vehicle_yaw
        if self.fixed_road is not None:
            frame, k = self.fixed_road[:2]
            axis = frame.yaw+math.atan(k)
        self.progress_fresh = self.course.observe(self.vehicle_x, self.vehicle_y, axis)
        if self.progress_fresh: self.total_distance = self.course.progress

    def control_loop(self, event):
        with self.state_lock:
            if self.shutting_down: return
            self.progress_fresh = False
            legacy.PathTrackingController.control_loop(self, event)
            if self.ebs_pub is None: return
            now = time.time()
            status = self.course.snapshot(self.progress_fresh)
            client = self.can_client
            status.update(stamp=now, mode=self.output_mode,
                          vehicle_output_enabled=self.enable_vehicle_output,
                          hardware_integration_required=True,
                          tracker_state=('stopped' if self.run_guard.stop_reason else
                                         ('tracking' if self.enable_vehicle_output else 'shadow_tracking')
                                         if self.is_started else 'waiting'),
                          software_stop_reason=self.run_guard.stop_reason,
                          pose=dict(x=self.vehicle_x, y=self.vehicle_y, yaw=self.vehicle_yaw),
                          estimated_speed_mps=self.vehicle_speed,
                          speed_source='GPS_pose_difference_filtered_clipped_not_wheel_speed',
                          target_speed_requirement_kmh=40.0,
                          other_command_publishers=self.other_command_publishers,
                          exclusive_vehicle_channel_verified=bool(self.can_client_ok and not self.other_command_publishers),
                          channel_check_scope='local_source_port_and_ROS_publishers_only',
                          throttle_command_implemented=False,
                          udp_frames_sent=client.sent_frames if client else 0,
                          udp_stop_frames_sent=client.sent_stop_frames if client else 0,
                          last_udp_send_ok=client.last_send_ok if client else None,
                          vcu_acknowledged=None,
                          road_age_s=(now-self.fixed_road[4]) if self.fixed_road else None,
                          software_steering_rad=self.prev_steering_angle)
            message = json.dumps(status, allow_nan=False)
            self.ebs_pub.publish(String(message))
            with self.journal_lock:
                if self.ebs_log is not None:
                    self.ebs_log.write(message+'\n')
                    self.ebs_log.flush()

    def shutdown(self):
        legacy.PathTrackingController.shutdown(self)
        with self.journal_lock:
            if self.ebs_log is not None:
                self.ebs_log.close()
                self.ebs_log = None

if __name__ == '__main__':
    node = EBSShadowController()
    rospy.on_shutdown(node.shutdown)
    try: rospy.spin()
    finally: node.shutdown()
