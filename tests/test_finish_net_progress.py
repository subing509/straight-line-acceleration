"""Exercise the actual finish method using only the standard library and AST.

No ROS imports, sockets, command publishers or physical vehicle outputs.
"""
from __future__ import division
import ast
import math
import os
import unittest


class Object(object):
    def __init__(self, **values):
        self.__dict__.update(values)


def load_subject(clock):
    source_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                              'src', 'acceleration_event', 'src')
    body = []
    for filename, name in [('legacy_run_guard.py', 'LegacyRunGuard'),
                           ('track_frame.py', 'finite')]:
        with open(os.path.join(source_dir, filename), 'rb') as stream:
            tree = ast.parse(stream.read(), filename)
        body.extend(node for node in tree.body
                    if isinstance(node, (ast.ClassDef, ast.FunctionDef))
                    and node.name == name)
    with open(os.path.join(source_dir, 'genzong_realtime.py'), 'rb') as stream:
        tree = ast.parse(stream.read(), 'genzong_realtime.py')
    methods = [method for cls in tree.body if isinstance(cls, ast.ClassDef)
               for method in cls.body if isinstance(method, ast.FunctionDef)
               and method.name == 'update_finish_distance']
    if len(body) != 2 or len(methods) != 1:
        raise AssertionError('Expected finish method and its validation helpers')
    isolated = ast.parse('')
    isolated.body = body + methods
    namespace = dict(math=math, time=Object(time=lambda: clock[0]))
    eval(compile(isolated, 'finish_net_progress_isolated', 'exec'), namespace)
    subject = type('Subject', (object,), {
        'update_finish_distance': namespace['update_finish_distance']})()
    subject.run_guard = namespace['LegacyRunGuard']()
    return subject


class FinishNetProgressTests(unittest.TestCase):
    def setUp(self):
        self.clock = [100.0]
        self.subject = load_subject(self.clock)
        subject = self.subject
        subject.is_started = subject.pose_received = False
        subject.finish_triggered = False
        subject.finish_distance = 75.0
        subject.start_x = subject.start_y = None
        subject.finish_axis_yaw = None
        subject.total_distance = 0.0
        subject.last_distance_x = subject.last_distance_y = None
        subject.vehicle_x = subject.vehicle_y = subject.vehicle_yaw = 0.0
        subject.fixed_road = (Object(yaw=0.0), 0.0, 0.0, 2, self.clock[0])

    def observe_all(self):
        for name in self.subject.run_guard.limits:
            self.subject.run_guard.observe(name, self.clock[0])

    def start(self, x=0.0, y=0.0, yaw=0.0):
        subject = self.subject
        subject.vehicle_x, subject.vehicle_y, subject.vehicle_yaw = x, y, yaw
        subject.pose_received = True
        self.observe_all()
        subject.is_started = subject.run_guard.start(self.clock[0])
        self.assertTrue(subject.is_started)
        subject.update_finish_distance()

    def pose(self, x, y=0.0, yaw=None):
        self.clock[0] += 0.05
        self.subject.vehicle_x, self.subject.vehicle_y = x, y
        if yaw is not None:
            self.subject.vehicle_yaw = yaw
        self.observe_all()
        self.subject.update_finish_distance()

    def test_missing_localization_cannot_freeze_default_pose(self):
        self.subject.is_started = self.subject.run_guard.running = True
        self.subject.update_finish_distance()
        self.assertIsNone(self.subject.start_x)
        self.assertIsNone(self.subject.finish_axis_yaw)

    def test_missing_pose_evidence_cannot_freeze_default_pose(self):
        self.subject.is_started = self.subject.run_guard.running = True
        self.subject.pose_received = True
        self.subject.update_finish_distance()
        self.assertIsNone(self.subject.start_x)

    def test_reference_is_recorded_only_after_existing_start_conditions(self):
        self.subject.pose_received = True
        self.subject.vehicle_x = 40.0
        self.observe_all()
        self.subject.update_finish_distance()
        self.assertIsNone(self.subject.start_x)
        self.start(x=41.0, y=-2.0)
        self.assertEqual((41.0, -2.0), (self.subject.start_x, self.subject.start_y))
        self.assertEqual(0.0, self.subject.total_distance)

    def test_stale_pose_cannot_freeze_reference_or_advance_progress(self):
        self.subject.pose_received = True
        self.observe_all()
        self.subject.is_started = self.subject.run_guard.start(self.clock[0])
        self.clock[0] += 0.9
        self.subject.update_finish_distance()
        self.assertIsNone(self.subject.start_x)
        self.pose(10.0)
        self.assertEqual(10.0, self.subject.start_x)
        self.clock[0] += 0.9
        self.subject.vehicle_x = 90.0
        self.subject.update_finish_distance()
        self.assertEqual(0.0, self.subject.total_distance)
        self.assertFalse(self.subject.finish_triggered)

    def test_source_age_is_included_in_existing_pose_timeout(self):
        self.start()
        self.subject.start_x = self.subject.start_y = None
        self.subject.finish_axis_yaw = None
        self.subject.run_guard.observe('pose', self.clock[0], source_age=0.9)
        self.subject.update_finish_distance()
        self.assertIsNone(self.subject.start_x)

    def test_nonfinite_pose_cannot_establish_reference(self):
        self.start()
        self.subject.start_x = self.subject.start_y = None
        self.subject.finish_axis_yaw = None
        self.pose(float('nan'))
        self.assertIsNone(self.subject.start_x)
        self.pose(0.0, yaw=float('inf'))
        self.assertIsNone(self.subject.start_x)

    def test_stationary_five_centimetre_noise_for_one_minute_does_not_finish(self):
        self.start()
        for index in range(1200):
            sign = 1.0 if index % 2 else -1.0
            self.pose(sign * 0.05, -sign * 0.05)
        self.assertLessEqual(abs(self.subject.total_distance), 0.05)
        self.assertFalse(self.subject.finish_triggered)

    def test_low_speed_reaches_original_seventy_five_metre_threshold(self):
        self.start(x=10.0, y=-2.0)
        for index in range(1, 3000):
            self.pose(10.0 + 0.5 * index / 20.0, -2.0)
            self.assertFalse(self.subject.finish_triggered)
        self.pose(85.0, -2.0)
        self.assertEqual(75.0, self.subject.total_distance)
        self.assertTrue(self.subject.finish_triggered)

    def test_cross_track_swing_does_not_count_as_forward_progress(self):
        frame_yaw, slope = math.radians(30.0), 0.2
        self.subject.fixed_road = (Object(yaw=frame_yaw), slope, 0.0, 2, self.clock[0])
        axis = frame_yaw + math.atan(slope)
        self.start(x=12.0, y=-2.0)
        for index in range(400):
            lateral = 20.0 if index % 2 else -20.0
            self.pose(12.0 - lateral * math.sin(axis), -2.0 + lateral * math.cos(axis))
        self.assertAlmostEqual(0.0, self.subject.total_distance)
        self.assertFalse(self.subject.finish_triggered)
        self.pose(12.0 + 10.0 * math.cos(axis), -2.0 + 10.0 * math.sin(axis))
        self.assertAlmostEqual(10.0, self.subject.total_distance)

    def test_reverse_motion_turns_and_new_road_fits_do_not_accumulate(self):
        self.start()
        for index, x in enumerate((40.0, 0.0, 40.0, -5.0)):
            self.subject.fixed_road = (Object(yaw=math.pi / 2), 0.5, 0.0, 2, self.clock[0])
            self.pose(x, yaw=index * math.pi / 2)
            self.assertAlmostEqual(x, self.subject.total_distance)
            self.assertFalse(self.subject.finish_triggered)
        self.assertEqual(0.0, self.subject.finish_axis_yaw)

    def test_repeated_control_ticks_do_not_advance_progress(self):
        self.start()
        self.pose(74.9)
        for unused in range(1000):
            self.subject.update_finish_distance()
        self.assertEqual(74.9, self.subject.total_distance)
        self.assertFalse(self.subject.finish_triggered)

    def test_heading_fallback_is_frozen_when_no_fixed_road_is_available(self):
        self.subject.fixed_road = None
        axis = math.radians(25.0)
        self.start(yaw=axis)
        self.subject.fixed_road = (Object(yaw=0.0), 0.0, 0.0, 2, self.clock[0])
        self.pose(10.0 * math.cos(axis), 10.0 * math.sin(axis), yaw=-math.pi)
        self.assertAlmostEqual(10.0, self.subject.total_distance)
        self.assertEqual(axis, self.subject.finish_axis_yaw)

    def test_stop_latch_prevents_reference_and_progress_changes(self):
        self.start()
        self.pose(10.0)
        self.subject.run_guard.stop('road_stale')
        self.pose(80.0)
        self.assertEqual(10.0, self.subject.total_distance)
        self.assertEqual('road_stale', self.subject.run_guard.stop_reason)
        self.subject.start_x = None
        self.subject.update_finish_distance()
        self.assertIsNone(self.subject.start_x)

    def test_finish_latch_persists_when_vehicle_returns_to_start(self):
        self.start()
        self.pose(75.0)
        self.pose(0.0)
        self.assertTrue(self.subject.finish_triggered)
        self.assertEqual(75.0, self.subject.total_distance)


if __name__ == '__main__':
    unittest.main()
