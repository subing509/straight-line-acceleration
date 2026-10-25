"""Controller IMU freshness gating with actual guards and isolated callback AST.

Only standard-library modules are loaded; no ROS, CAN or vehicle outputs.
"""
import ast
import os
import threading
import unittest


class Object(object):
    def __init__(self, **values):
        self.__dict__.update(values)


SOURCE_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                          'src', 'acceleration_event', 'src')


def pure_module(filename):
    path = os.path.join(SOURCE_DIR, filename)
    with open(path, 'rb') as stream:
        source = stream.read()
    namespace = {'__name__': 'isolated_' + filename.replace('.', '_')}
    eval(compile(source, path, 'exec'), namespace)
    return namespace


def controller_callback(namespace):
    path = os.path.join(SOURCE_DIR, 'genzong_realtime.py')
    with open(path, 'rb') as stream:
        tree = ast.parse(stream.read(), path)
    methods = [method for cls in tree.body if isinstance(cls, ast.ClassDef)
               for method in cls.body if isinstance(method, ast.FunctionDef)
               and method.name == 'guard_imu_callback']
    if len(methods) != 1:
        raise AssertionError('Expected exactly one controller IMU callback')
    isolated = ast.parse('')
    isolated.body = methods
    eval(compile(isolated, path, 'exec'), namespace)
    return type('Subject', (object,), {
        'guard_imu_callback': namespace['guard_imu_callback']})()


class ControllerImuIntegrityTests(unittest.TestCase):
    def setUp(self):
        self.ros_now, self.wall_now = 100.0, 1000.0
        namespace = dict(
            rospy=Object(Time=Object(now=lambda: Object(to_sec=lambda: self.ros_now))),
            time=Object(time=lambda: self.wall_now))
        self.subject = controller_callback(namespace)
        self.subject.state_lock = threading.Lock()
        self.subject.imu_integrity = pure_module('heading_integrity.py')['HeadingIntegrityGuard']()
        self.subject.run_guard = pure_module('legacy_run_guard.py')['LegacyRunGuard']()

    def deliver(self, heading, stamp=None, advance=0.02):
        self.ros_now += advance
        self.wall_now += advance
        if stamp is None:
            stamp = self.ros_now
        msg = Object(orientation=Object(z=heading),
                     header=Object(stamp=Object(to_sec=lambda: stamp)))
        self.subject.guard_imu_callback(msg)

    def warm(self, headings=(0.0, 0.0, 0.0)):
        for heading in headings:
            self.deliver(heading)
        self.assertIn('imu', self.subject.run_guard.samples)

    def test_stationary_start_confirms_after_three_frames_without_movement(self):
        self.deliver(0.0)
        self.assertNotIn('imu', self.subject.run_guard.samples)
        self.deliver(0.0)
        self.assertNotIn('imu', self.subject.run_guard.samples)
        self.deliver(0.0)
        self.assertEqual((self.wall_now, 0.0), self.subject.run_guard.samples['imu'])

    def test_physical_turn_and_heading_wrap_refresh_imu_evidence(self):
        self.warm((179.0, -179.0, -178.0))
        before = self.subject.run_guard.samples['imu']
        self.deliver(179.0)
        after = self.subject.run_guard.samples['imu']
        self.assertGreater(after[0], before[0])
        self.assertEqual(0.0, after[1])

    def test_single_large_spike_does_not_refresh_and_next_normal_frame_recovers(self):
        self.warm()
        accepted = self.subject.run_guard.samples['imu']
        self.deliver(157.0)
        self.assertEqual(accepted, self.subject.run_guard.samples['imu'])
        self.deliver(1.0)
        self.assertGreater(self.subject.run_guard.samples['imu'][0], accepted[0])

    def test_nonfinite_headings_do_not_refresh_evidence(self):
        self.warm()
        accepted = self.subject.run_guard.samples['imu']
        for value in (float('nan'), float('inf'), -float('inf')):
            self.deliver(value)
            self.assertEqual(accepted, self.subject.run_guard.samples['imu'])

    def test_stale_future_zero_duplicate_and_out_of_order_stamps_do_not_refresh(self):
        self.warm()
        accepted = self.subject.run_guard.samples['imu']
        last_source = self.ros_now
        for stamp in (last_source - 1.0, last_source + 1.0, 0.0,
                      last_source, last_source - 0.01):
            self.deliver(0.0, stamp=stamp)
            self.assertEqual(accepted, self.subject.run_guard.samples['imu'])

    def test_accepted_source_age_is_preserved_for_existing_guard_timeout(self):
        self.warm()
        self.deliver(1.0, stamp=self.ros_now + 0.01)
        observed, source_age = self.subject.run_guard.samples['imu']
        self.assertEqual(self.wall_now, observed)
        self.assertAlmostEqual(0.01, source_age)

    def test_continuous_bad_frames_trigger_existing_imu_stale_and_do_not_unlatch(self):
        self.warm()
        guard = self.subject.run_guard
        for name in ('pose', 'path', 'road'):
            guard.observe(name, self.wall_now)
        self.assertTrue(guard.start(self.wall_now))
        accepted = guard.samples['imu']
        for unused in range(41):
            self.deliver(157.0)
            for name in ('pose', 'path', 'road'):
                guard.observe(name, self.wall_now)
            guard.evaluate(self.wall_now)
        self.assertEqual(accepted, guard.samples['imu'])
        self.assertEqual('imu_stale', guard.stop_reason)
        self.deliver(0.0)
        self.assertEqual('imu_stale', guard.stop_reason)
        self.assertFalse(guard.start(self.wall_now))


if __name__ == '__main__':
    unittest.main()
