# -*- coding: utf-8 -*-
"""Actual planner geometry with the deployed launch profile; no ROS/CAN output."""
from __future__ import division
import ast
import math
import os
import unittest
import xml.etree.ElementTree as ET
import test_track_frame_nodes as fixtures
from legacy_run_guard import LegacyRunGuard
from track_frame import TrackFrame

ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))

def profile():
    node=next(n for n in ET.parse(os.path.join(ROOT,'launch','acceleration_event.launch')).getroot().findall('node')
              if n.get('name')=='straight_line_planner_realtime')
    return {p.get('name'):float(p.get('value')) for p in node.findall('param')}

class TrackToleranceTests(unittest.TestCase):
    def planner(self,width=3.15):
        self.helper=fixtures.TrackGeometryTests('test_stationary_one_row_initializes_without_motion_or_multi_frame_gate')
        p=self.helper.planner(offset=0.,rows=(5,))
        p.centerline_reference_k=None
        p.__dict__.update({k:profile()[k] for k in ('track_width','pair_distance_min','pair_distance_max','straight_slope_limit')})
        p.cones=[(5.,y,1.) for y in (-width/2,width/2)]
        return p

    def observe(self,p,width,slope):
        self.helper.now+=.1;p.last_cone_time=self.helper.now
        p.cones=[(x,y+slope*x,1.) for x in (5.,10.) for y in (-width/2,width/2)]
        return p.generate_local_path()

    def test_track_width_endpoints_and_midpoint_start_without_motion(self):
        for width in (2.7,3.15,3.6):
            p=self.planner(width)
            self.assertTrue(p.generate_local_path()[1])
            self.assertTrue(self.observe(p,width,.0)[2])
            self.assertEqual(2,p.centerline_pairs)

    def test_width_endpoints_with_maximum_row_skew_are_paired(self):
        for width in (2.7,3.6):
            p=self.planner(width);p.track_frame=TrackFrame(0,0,0)
            p.cones=[(x,y,1.) for base in (0.,4.) for x,y in ((base,-width/2),(base+.8,width/2))]
            rows=p.pair_cones(p.select_local_cones())
            self.assertEqual(2,len(rows))
            self.assertAlmostEqual(rows[0][2],math.hypot(width,.8))

    def test_widths_outside_measurement_margin_are_rejected(self):
        for width in (2.4,3.9):
            p=self.planner(width)
            self.assertFalse(p.generate_local_path()[1])

    def test_large_longitudinal_mismatch_still_rejected(self):
        p=self.planner();p.track_frame=TrackFrame(0,0,0)
        p.cones=[(5.,-1.5,1.),(5.9,1.5,1.)]
        self.assertEqual([],p.pair_cones(p.select_local_cones()))

    def test_recorded_startup_slopes_old_profile_stops_new_profile_runs(self):
        for limit,expected in ((.06,'road_stale'),(profile()['straight_slope_limit'],None)):
            p=self.planner();p.straight_slope_limit=limit;p.generate_local_path()
            guard=LegacyRunGuard()
            for name in guard.limits:guard.observe(name,self.helper.now)
            self.assertTrue(guard.start(self.helper.now))
            # Synthetic complete rows reconstruct slopes from the actual startup log.
            # They are not raw recorded cone coordinates.
            for i in range(25):
                self.observe(p,3.15,(.0699,.0704,.0709,.0686)[i%4])
                for name in ('pose','imu','path'):guard.observe(name,self.helper.now)
                guard.observe('road',self.helper.now,self.helper.now-p.centerline_last_valid_time)
                guard.evaluate(self.helper.now)
            self.assertEqual(expected,guard.stop_reason)

    def test_first_two_row_direction_above_new_limit_still_rejected(self):
        p=self.planner();p.generate_local_path()
        self.assertFalse(self.observe(p,3.15,.11)[2])
        self.assertIsNone(p.centerline_reference_k)

    def test_fixed_direction_reference_does_not_follow_drift(self):
        p=self.planner();p.generate_local_path()
        self.assertTrue(self.observe(p,3.15,.08)[2])
        self.assertTrue(self.observe(p,3.15,.16)[2])
        support=p.centerline_last_valid_time
        self.assertFalse(self.observe(p,3.15,.20)[2])
        self.assertAlmostEqual(.08,p.centerline_reference_k)
        self.assertEqual(support,p.centerline_last_valid_time)

    def test_missing_road_still_stops_with_original_timeout(self):
        p=self.planner();p.generate_local_path();guard=LegacyRunGuard()
        for name in guard.limits:guard.observe(name,self.helper.now)
        guard.start(self.helper.now)
        self.helper.now+=1.1
        for name in ('pose','imu','path'):guard.observe(name,self.helper.now)
        guard.observe('road',self.helper.now,self.helper.now-p.centerline_last_valid_time)
        guard.evaluate(self.helper.now)
        self.assertEqual('road_stale',guard.stop_reason)

    def test_linux_planner_shebang_has_no_carriage_return(self):
        with open(os.path.join(ROOT,'src','zhixian_realtime.py'),'rb') as stream:
            self.assertEqual(b'#!/usr/bin/env python\n',stream.readline())

    def test_manual_planner_defaults_match_launch_profile(self):
        tree=ast.parse(open(os.path.join(ROOT,'src','zhixian_realtime.py'),'rb').read())
        values={}
        for node in ast.walk(tree):
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='get_param' and len(node.args)==2:
                try:values[ast.literal_eval(node.args[0]).lstrip('~')]=ast.literal_eval(node.args[1])
                except (ValueError,TypeError):pass
        for key in ('track_width','pair_distance_min','pair_distance_max','straight_slope_limit'):
            self.assertEqual(profile()[key],values[key])

if __name__=='__main__':unittest.main()
