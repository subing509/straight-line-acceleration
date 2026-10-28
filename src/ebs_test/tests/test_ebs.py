# -*- coding: utf-8 -*-
from __future__ import division
import ast
import json
import math
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ebs_test/scripts'))
sys.path.insert(0, str(ROOT/'acceleration_event/tests'))
from ebs_core import CourseProgress, require_shadow_only
from ebs_output import validate_output_request, exclusive_client_class
import socket
from test_lidar_fusion_node import load_node, Obj, Stamp

class EBSCoreTests(unittest.TestCase):
    def test_never_infers_res_or_standstill_from_regions(self):
        p=CourseProgress();p.observe(0,0,0)
        for x,region in [(0,'acceleration_region'),(24.99,'acceleration_region'),
                         (25,'braking_region'),(35,'braking_region'),(75,'beyond_course')]:
            p.observe(x,0,1.2);s=p.snapshot(True)
            self.assertEqual(region,s['geometric_region'])
            for key in ('hardware_go','hardware_ebs_active','hardware_emergency',
                        'hardware_standstill','test_passed','actual_speed_mps'):
                self.assertIsNone(s[key])
            self.assertFalse(s['region_is_brake_trigger'])

    def test_fixed_axis_net_progress_not_accumulated_distance(self):
        p=CourseProgress();p.observe(10,20,math.pi/2)
        p.observe(100,30,0);self.assertAlmostEqual(10,p.progress)
        p.observe(10,25,0);self.assertAlmostEqual(5,p.progress)

    def test_invalid_and_stale_pose_do_not_commit(self):
        p=CourseProgress()
        self.assertFalse(p.observe(0,0,float('nan')))
        self.assertFalse(p.observe(0,0,0,False));self.assertIsNone(p.origin)
        p.observe(0,0,0);p.observe(20,0,0,False)
        self.assertEqual(0,p.progress)
        self.assertEqual('unknown',p.snapshot(False)['geometric_region'])

    def test_invalid_course_lengths(self):
        for n in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):CourseProgress(n,10)

    def test_every_output_request_is_refused(self):
        require_shadow_only(False)
        for request in (True,1,'true'):
            with self.assertRaises(RuntimeError):require_shadow_only(request)

class ActualTrackerAdapterTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        module,self.clock,_=load_node()
        base=module.LidarTrackingController.__bases__[0]
        self.ros=base.__init__.__globals__['rospy']
        noop=lambda *a,**kw:None
        for name in ('loginfo_throttle','logwarn_throttle','logerr_throttle','logerr','logdebug','logdebug_throttle'):
            setattr(self.ros,name,noop)
        self.ros.get_master=lambda:Obj(getSystemState=lambda:(1,'',[[],[],[]]))
        self.ros.get_name=lambda:'/ebs_test/controller'
        self.clock.params={'~enable_vehicle_output':False,'~log_dir':self.temp.name}
        tree=ast.parse((ROOT/'ebs_test/scripts/ebs_controller.py').read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        self.actual_can = base.configure_outputs.__globals__['CANUDPClient']
        scope=dict(legacy=Obj(PathTrackingController=base,CANUDPClient=self.actual_can),CourseProgress=CourseProgress,
                   require_shadow_only=require_shadow_only,rospy=self.ros,
                   validate_output_request=validate_output_request,exclusive_client_class=exclusive_client_class,
                   threading=threading,time=time,os=os,math=math,json=json,
                   String=base.__init__.__globals__['String'])
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),
                     'actual_ebs_adapter','exec'),scope)
        self.cls=scope['EBSShadowController']
        self.node=None

    def tearDown(self):
        if self.node:self.node.shutdown()
        self.temp.cleanup()

    def node_ready(self):
        self.node=self.cls();return self.node

    def test_disabled_creates_no_socket_and_no_simulator_command(self):
        n=self.node_ready()
        self.assertIsNone(n.can_client);self.assertFalse(n.can_client_ok)
        self.assertIsNone(n.cmd_pub)
        n.refresh_output_readiness();self.assertIsNone(n.can_client)

    def test_output_true_fails_before_can_constructor(self):
        self.clock.params['~enable_vehicle_output']=True
        with self.assertRaises(RuntimeError):self.cls()

    def enable_loopback(self):
        self.receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.receiver.bind(('127.0.0.1',0));self.receiver.settimeout(1)
        self.addCleanup(self.receiver.close)
        self.clock.params.update({'~enable_vehicle_output':True,
                                 '~output_mode':'original_protocol_test',
                                 '~can_dest_ip':'127.0.0.1',
                                 '~can_dest_port':self.receiver.getsockname()[1],
                                 '~can_self_port':0})
        return self.node_ready()

    def test_original_codec_and_guard_stop_reach_loopback(self):
        n=self.enable_loopback()
        self.assertIsNone(n.cmd_pub);self.assertFalse(n.fssim_available)
        self.assertTrue(n.can_client.send_steering_frame(0.0))
        packet=self.receiver.recv(100)
        self.assertEqual(bytes.fromhex('0800000210f401000000000000'),packet)
        self.assertTrue(n.send_guard_stop())
        self.assertEqual(bytes.fromhex('0800000210d007000000000000'),self.receiver.recv(100))
        n.shutdown()
        self.assertEqual(2000,int.from_bytes(self.receiver.recv(100)[5:7],'little'))

    def test_port_cannot_be_shared_with_original_reuse_sender(self):
        n=self.enable_loopback()
        port=n.can_client.sock.getsockname()[1]
        contender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.addCleanup(contender.close)
        contender.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        with self.assertRaises(OSError):contender.bind(('',port))

    def test_existing_sender_prevents_transport_creation(self):
        occupied=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.addCleanup(occupied.close)
        occupied.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        occupied.bind(('',0))
        self.clock.params.update({'~enable_vehicle_output':True,
            '~output_mode':'original_protocol_test','~can_dest_ip':'127.0.0.1',
            '~can_self_port':occupied.getsockname()[1]})
        with self.assertRaises(OSError):self.cls()

    def test_competing_command_publisher_refused_before_transport(self):
        self.clock.params.update({'~enable_vehicle_output':True,
                                 '~output_mode':'original_protocol_test'})
        self.ros.get_master=lambda:Obj(getSystemState=lambda:(1,'',[
            [('/control/steering',['/other'])],[],[]]))
        with self.assertRaises(RuntimeError):self.cls()

    def test_runtime_conflict_latches_stop_and_does_not_self_conflict(self):
        n=self.enable_loopback()
        self.ros.get_master=lambda:Obj(getSystemState=lambda:(1,'',[
            [('/control/steering',['/ebs_test/controller'])],[],[]]))
        n.last_conflict_check=0;n.refresh_output_readiness()
        self.assertIsNone(n.run_guard.stop_reason)
        self.ros.get_master=lambda:Obj(getSystemState=lambda:(1,'',[
            [('/control/steering',['/ebs_test/controller','/other'])],[],[]]))
        n.run_guard.running=True;n.is_started=True
        n.last_conflict_check=0;n.control_loop(None)
        self.assertEqual('command_channel_conflict',n.run_guard.stop_reason)
        self.assertEqual(2000,int.from_bytes(self.receiver.recv(100)[5:7],'little'))

    def test_output_profile_rejects_highspeed_candidate(self):
        self.clock.params.update({'~enable_vehicle_output':True,
            '~output_mode':'original_protocol_test','~profile':'highspeed_candidate'})
        with self.assertRaises(RuntimeError):self.cls()

    def test_runtime_mutation_also_fails_before_retry(self):
        n=self.node_ready();n.enable_vehicle_output=True
        with self.assertRaises(RuntimeError):n.refresh_output_readiness()
        self.assertIsNone(n.can_client)
        n.enable_vehicle_output=False

    def test_no_finish_at_25_35_or_75m(self):
        n=self.node_ready();n.is_started=True;n.pose_received=True;n.run_guard.running=True
        n.vehicle_x=n.vehicle_y=n.vehicle_yaw=0
        for x in (0,25,35,75,100):
            n.run_guard.observe('pose',time.time());n.vehicle_x=x
            n.update_finish_distance()
            self.assertFalse(n.finish_triggered)
        self.assertEqual(100,n.total_distance)
        n.run_guard.samples['pose']=(time.time()-2,0)
        n.vehicle_x=200;n.update_finish_distance()
        self.assertFalse(n.progress_fresh);self.assertEqual(100,n.total_distance)

    def test_shadow_status_and_conflict_warning_do_not_claim_hardware_state(self):
        n=self.node_ready()
        self.ros.get_master=lambda:Obj(getSystemState=lambda:(1,'',[
            [('/control/steering',['/original_tracker'])],[],[]]))
        n.control_loop(None)
        status=json.loads(n.ebs_pub.messages[-1].data)
        self.assertEqual(['/original_tracker'],status['other_command_publishers'])
        self.assertEqual('shadow_only',status['mode'])
        self.assertIsNone(status['hardware_go'])
        self.assertFalse(status['vehicle_output_enabled'])
        self.assertIsNone(n.can_client)
        n.shutdown()
        self.assertIsNone(n.ebs_log)

    def test_no_fssim_pose_override(self):
        n=self.node_ready();n.fssim_pose_callback(object())
        self.assertFalse(n.fssim_pose_received)

    def test_road_loss_latches_without_any_hardware_output(self):
        n=self.node_ready();now=time.time()
        for name in n.run_guard.limits:n.run_guard.observe(name,now)
        self.assertTrue(n.run_guard.start(now));n.is_started=True
        n.run_guard.evaluate(now+1.1)
        self.assertEqual('pose_stale',n.run_guard.stop_reason)
        # Specifically isolate road failure too.
        n.run_guard.stop_reason=None
        for name in ('pose','imu','path'):n.run_guard.observe(name,now+1.1)
        n.run_guard.evaluate(now+1.1)
        self.assertEqual('road_stale',n.run_guard.stop_reason)
        self.assertFalse(n.send_guard_stop());self.assertIsNone(n.can_client)

class LaunchContractTests(unittest.TestCase):
    def test_business_topics_isolated_and_no_sensor_driver_or_old_launch(self):
        import xml.etree.ElementTree as ET
        tree=ET.parse(str(ROOT/'ebs_test/launch/ebs_shadow.launch')).getroot()
        defaults={a.get('name'):a.get('default') for a in tree.findall('arg')}
        self.assertEqual('false',defaults['enable_vehicle_output'])
        self.assertEqual('baseline_shadow',defaults['profile'])
        self.assertEqual([],tree.findall('.//include'))
        group=tree.find('group');self.assertEqual('ebs_test',group.get('ns'))
        nodes=group.findall('node');self.assertEqual(3,len(nodes))
        for node in nodes:
            remaps={r.get('from'):r.get('to') for r in node.findall('remap')}
            for name in ('/vehicle_pose','/cone_map','/planned_path',
                         '/control/legacy_status','/control/legacy_candidate_steering'):
                self.assertTrue(remaps[name].startswith('/ebs_test/'))
            self.assertNotIn('/GPS_data',remaps)
            self.assertNotIn('/imu_data',remaps)
        ctrl=next(n for n in nodes if n.get('name')=='controller')
        params={p.get('name'):p.get('value') for p in ctrl.findall('param')}
        self.assertEqual('$(arg enable_vehicle_output)',params['enable_vehicle_output'])
        self.assertEqual('192.168.0.7',defaults['can_dest_ip'])
        self.assertEqual('20005',defaults['can_dest_port'])
        self.assertEqual('$(arg can_dest_ip)',params['can_dest_ip'])
        self.assertEqual('$(arg can_dest_port)',params['can_dest_port'])

    def test_profiles_do_not_claim_software_brake_or_fixed_throttle(self):
        import yaml
        base=yaml.safe_load((ROOT/'ebs_test/config/baseline_shadow.yaml').read_text())
        ctrl=base['controller']
        self.assertNotIn('finish_distance',ctrl)
        self.assertNotIn('fixed_speed',ctrl)
        self.assertGreater(ctrl['max_vehicle_speed_mps'],40/3.6)
        self.assertGreater(base['mapper']['gps_max_speed_mps'],40/3.6)
        self.assertEqual(1.0,ctrl['guard_road_timeout'])
        candidate=yaml.safe_load((ROOT/'ebs_test/config/highspeed_candidate.yaml').read_text())
        c=candidate['controller']
        preview=min(c['lookahead_max'],c['lookahead_min']+c['lookahead_speed_gain']*40/3.6)
        self.assertAlmostEqual(9.555556,preview,places=5)

if __name__=='__main__':unittest.main()
