"""Run actual mapper callbacks without ROS, GUI, sockets or vehicle output."""
from __future__ import print_function
import ast
import copy
import json
import math
import os
import sys
import threading
import unittest

SOURCE = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                      'src', 'acceleration_event', 'src')
sys.path.insert(0, SOURCE)
from gps_integrity import GpsIntegrityGuard, finite
from heading_integrity import HeadingIntegrityGuard


class Obj(object):
    def __init__(self, **values):
        self.__dict__.update(values)


class Stamp(object):
    current = 100.0

    def __init__(self, value=0):
        self.value = value

    def to_sec(self):
        return self.value

    def __sub__(self, other):
        return Stamp(self.value-other.value)

    @classmethod
    def now(cls):
        return cls(cls.current)


class Publisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


def pose():
    return Obj(header=Obj(), pose=Obj(position=Obj(), orientation=Obj()))


def classes():
    filename = os.path.join(SOURCE, 'jiantu.py')
    with open(filename, 'rb') as stream:
        tree = ast.parse(stream.read(), filename)
    module = ast.parse('')
    module.body = [node for node in tree.body if isinstance(node, ast.ClassDef)
                   and node.name in ('CoordinateTransformer', 'FSAESLAMNode')]
    rospy = Obj(Time=Stamp, loginfo=lambda *args: None,
                logwarn_throttle=lambda *args: None)
    namespace = dict(rospy=rospy, math=math, json=json,
                     np=Obj(all=all, isfinite=lambda xs: [finite(x) for x in xs]),
                     Header=Obj, PoseStamped=pose, String=lambda value: value)
    eval(compile(module, filename, 'exec'), namespace)
    return namespace


class MapperGpsTests(unittest.TestCase):
    def setUp(self):
        Stamp.current = 100.0
        namespace = classes()
        self.node = namespace['FSAESLAMNode'].__new__(namespace['FSAESLAMNode'])
        n = self.node
        n.state_lock, n.cone_lock = threading.Lock(), threading.Lock()
        n.gps_guard, n.last_gps_stamp = GpsIntegrityGuard(), Stamp()
        n.imu_integrity = HeadingIntegrityGuard()
        for stamp in (99.96, 99.98, 100.):
            n.imu_integrity.accept(0., stamp, stamp)
        n.gps_msg_count, n.imu_msg_count = 0, 0
        n._is_initialized = False
        n.init_start_time, n.imu_init_wait_time = Stamp(97.0), 3.0
        n.current_lat, n.current_lon, n.current_yaw = None, None, 0.0
        n.current_x, n.current_y = 0.0, 0.0
        n.origin_lat, n.origin_lon, n.origin_yaw = None, None, None
        n.gps_lat_min, n.gps_lat_max, n.gps_lon_min, n.gps_lon_max = 30, 40, 110, 125
        n.transformer = namespace['CoordinateTransformer']()
        n.vehicle_pose_pub, n.gps_status_pub = Publisher(), Publisher()
        n.imu_status_pub = Publisher()
        n.cone_detection_count = 0
        n.transform_lidar_to_vehicle = lambda x, y: (x, y)
        n.is_vehicle_return = lambda x, y: False

    def gps(self, value, lon=116.1993085):
        Stamp.current = value
        self.node.gps_callback(Obj(latitude=39.9260694, longitude=lon,
                                   header=Obj(stamp=Stamp(value))))

    def initialize(self):
        for value in (100.0, 100.02, 100.04, 100.06):
            self.gps(value)
        self.assertTrue(self.node._is_initialized)

    def test_stationary_can_initialize_without_moving(self):
        self.initialize()
        self.assertEqual(self.node.origin_lon, 116.1993085)
        self.assertEqual(self.node.current_x, 0.0)
        self.assertTrue(self.node.vehicle_pose_pub.messages)

    def test_rejected_spike_does_not_move_pose_or_origin_and_next_fix_recovers(self):
        self.initialize()
        n = self.node
        before = (n.current_lat, n.current_lon, n.current_x, n.current_y,
                  n.origin_lat, n.origin_lon, n.last_gps_stamp.to_sec())
        count = len(n.vehicle_pose_pub.messages)
        self.gps(100.08, lon=111.7486336)
        after = (n.current_lat, n.current_lon, n.current_x, n.current_y,
                 n.origin_lat, n.origin_lon, n.last_gps_stamp.to_sec())
        self.assertEqual(before, after)
        self.assertEqual(count, len(n.vehicle_pose_pub.messages))
        self.gps(100.1)
        self.assertEqual(count+1, len(n.vehicle_pose_pub.messages))
        self.assertEqual(100.1, n.vehicle_pose_pub.messages[-1].header.stamp.to_sec())

    def test_pose_preserves_measurement_time_and_never_refreshes_expired_fix(self):
        self.initialize()
        count = len(self.node.vehicle_pose_pub.messages)
        Stamp.current = 100.25
        self.node.publish_vehicle_state()
        self.assertEqual(100.06, self.node.vehicle_pose_pub.messages[-1].header.stamp.to_sec())
        Stamp.current = 101.0
        self.node.publish_vehicle_state()
        self.assertEqual(count+1, len(self.node.vehicle_pose_pub.messages))

    def test_imu_alone_cannot_initialize_from_old_gps(self):
        self.node.init_start_time = None
        for value in (100.0, 100.02, 100.04):
            self.gps(value)
        Stamp.current = 104.0
        self.node.imu_callback(Obj(orientation=Obj(z=0.0), header=Obj(stamp=Stamp(104.))))
        self.assertFalse(self.node._is_initialized)

    def test_lidar_keeps_preview_but_does_not_merge_with_expired_gps(self):
        self.initialize()
        Stamp.current = 101.0
        self.node._merge_cone_unlocked = lambda *args: self.fail('Expired pose entered map')
        self.node.cone_callback(Obj(poses=[Obj(position=Obj(x=5., y=1.5, z=-.4))]))
        self.assertEqual(self.node.last_lidar_accepted, 1)
        self.assertEqual(self.node.last_lidar_points, [(5., 1.5)])

    def test_diagnostic_reports_actual_accepted_age(self):
        self.initialize()
        Stamp.current = 100.3
        self.node.publish_gps_status(None)
        status = json.loads(self.node.gps_status_pub.messages[-1])
        self.assertEqual(status['accepted_stamp'], 100.06)
        self.assertAlmostEqual(status['accepted_age_s'], .24)
        self.assertEqual(status['counts']['accepted'], 2)

    def test_heading_spike_does_not_rotate_pose_and_next_normal_heading_recovers(self):
        self.initialize()
        Stamp.current = 100.08
        self.node.imu_callback(Obj(orientation=Obj(z=157.), header=Obj(stamp=Stamp.now())))
        self.assertEqual(0., self.node.current_yaw)
        self.assertEqual(100., self.node.imu_integrity.stamp)
        Stamp.current = 100.1
        self.node.imu_callback(Obj(orientation=Obj(z=1.), header=Obj(stamp=Stamp.now())))
        self.assertEqual(1., self.node.current_yaw)

    def test_fresh_gps_cannot_hide_expired_heading(self):
        self.initialize()
        count = len(self.node.vehicle_pose_pub.messages)
        self.gps(100.7)
        self.assertEqual(count, len(self.node.vehicle_pose_pub.messages))


if __name__ == '__main__':
    unittest.main()
