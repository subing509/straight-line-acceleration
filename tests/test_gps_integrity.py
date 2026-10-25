# -*- coding: utf-8 -*-
"""GPS position continuity tests; no ROS, serial port, or vehicle output."""
from __future__ import print_function

import math
import os
import sys
import unittest


MODULE_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                          'src', 'acceleration_event', 'src')
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)
from gps_integrity import GpsIntegrityGuard


LATITUDE = 39.9260694
LONGITUDE = 116.1993085
EARTH_RADIUS = 6371000.0


def position(east_m=0.0, north_m=0.0):
    """Construct nearby positions in metres at the recorded test location."""
    latitude = LATITUDE + math.degrees(north_m / EARTH_RADIUS)
    longitude = LONGITUDE + math.degrees(
        east_m / (EARTH_RADIUS * math.cos(math.radians(LATITUDE))))
    return latitude, longitude


class GpsIntegrityTest(unittest.TestCase):
    def setUp(self):
        self.guard = GpsIntegrityGuard()

    def accept_position(self, east_m, north_m, stamp, now=None):
        lat, lon = position(east_m, north_m)
        return self.guard.accept(lat, lon, stamp, stamp if now is None else now)

    def bootstrap(self, start=100.0):
        for number in range(2):
            self.accept_position(0.0, 0.0, start + number * 0.02)
            self.assertFalse(self.guard.ready)
        self.assertTrue(self.accept_position(0.0, 0.0, start + 0.04))
        self.assertTrue(self.guard.ready)
        self.assertTrue(self.guard.is_fresh(start + 0.04))
        return start + 0.04

    def anchor(self):
        return self.guard.latitude, self.guard.longitude, self.guard.stamp

    def test_stationary_startup_and_updates_do_not_require_movement(self):
        self.assertFalse(self.guard.ready)
        self.assertIsNone(self.guard.stamp)
        self.assertFalse(self.guard.is_fresh(100.0))
        stamp = self.bootstrap()
        for number in range(1, 101):
            self.assertTrue(self.accept_position(0, 0, stamp + number * 0.02))
        self.assertEqual(self.guard.latitude, LATITUDE)
        self.assertEqual(self.guard.longitude, LONGITUDE)
        self.assertEqual(self.guard.counts['received'], 103)
        self.assertIsInstance(self.guard.snapshot(stamp + 2.0), dict)

    def test_normal_15_kmh_motion_is_accepted_without_rate_limit(self):
        stamp = self.bootstrap()
        speed = 4.1667
        for number in range(1, 251):
            elapsed = number * 0.02
            self.assertTrue(self.accept_position(speed * elapsed, 0,
                                                 stamp + elapsed))
        self.assertAlmostEqual(self.guard.longitude,
                               position(speed * 5.0, 0)[1], places=10)
        self.assertTrue(self.guard.is_fresh(stamp + 5.0))

    def test_continuous_turning_is_accepted(self):
        stamp = self.bootstrap()
        radius, speed = 5.0, 4.1667
        for number in range(1, 201):
            elapsed = number * 0.05
            angle = speed * elapsed / radius
            east = radius * math.sin(angle)
            north = radius * (1.0 - math.cos(angle))
            self.assertTrue(self.accept_position(east, north, stamp + elapsed))

    def test_bad_first_fix_can_reseed_from_three_normal_samples(self):
        self.guard.accept(LATITUDE, 111.7486336, 100.0, 100.0)
        self.assertFalse(self.guard.ready)
        for stamp in (100.02, 100.04):
            self.accept_position(0, 0, stamp)
            self.assertFalse(self.guard.ready)
        self.assertTrue(self.accept_position(0, 0, 100.06))
        self.assertTrue(self.guard.ready)
        self.assertEqual(self.guard.latitude, LATITUDE)
        self.assertEqual(self.guard.longitude, LONGITUDE)

    def test_each_recorded_in_range_spike_keeps_anchor_and_recovers(self):
        stamp = self.bootstrap()
        for number, bad_longitude in enumerate((111.7486336, 111.7486336,
                                                111.7454461, 111.7454461)):
            now = stamp + (number * 2 + 1) * 0.02
            anchor = self.anchor()
            self.assertFalse(self.guard.accept(LATITUDE, bad_longitude, now, now))
            self.assertEqual(self.anchor(), anchor)
            self.assertTrue(self.guard.ready)
            self.assertTrue(self.accept_position(0, 0, now + 0.02))

    def test_repeated_bad_positions_never_refresh_last_accepted_time(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        for number in range(1, 101):
            now = stamp + number * 0.02
            self.assertFalse(self.guard.accept(LATITUDE, 111.7486336, now, now))
        self.assertEqual(self.anchor(), anchor)
        self.assertTrue(self.guard.ready)
        self.assertFalse(self.guard.is_fresh(stamp + 2.0))
        self.assertGreaterEqual(self.guard.counts['rejected'], 100)
        self.assertTrue(self.accept_position(0, 0, stamp + 2.02))
        self.assertTrue(self.guard.is_fresh(stamp + 2.02))

    def test_duplicate_out_of_order_zero_delayed_and_future_stamps_reject(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        cases = ((stamp, stamp + 0.02),
                 (stamp - 0.01, stamp + 0.02),
                 (0.0, stamp + 0.02),
                 (-1.0, stamp + 0.02),
                 (stamp + 0.02, stamp + 0.62),
                 (stamp + 1.0, stamp + 0.02))
        for source, receipt in cases:
            self.assertFalse(self.accept_position(0, 0, source, receipt))
            self.assertEqual(self.anchor(), anchor)
            self.assertTrue(self.guard.ready)
        self.assertTrue(self.accept_position(0, 0, stamp + 0.04))

    def test_freshness_uses_last_accepted_source_stamp(self):
        stamp = self.bootstrap()
        self.assertTrue(self.guard.is_fresh(stamp + 0.49))
        self.assertFalse(self.guard.is_fresh(stamp + 0.51))
        self.assertFalse(self.accept_position(0, 0, stamp + 0.02, stamp + 0.62))
        self.assertEqual(self.guard.stamp, stamp)
        self.assertFalse(self.guard.is_fresh(stamp + 0.62))

    def test_clock_rewind_does_not_clear_or_move_anchor(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        self.assertFalse(self.guard.is_fresh(stamp - 0.5))
        self.assertFalse(self.accept_position(0, 0, stamp - 0.5, stamp - 0.5))
        self.assertEqual(self.anchor(), anchor)
        self.assertTrue(self.guard.ready)
        self.assertTrue(self.accept_position(0.1, 0, stamp + 0.02))
        self.assertTrue(self.guard.is_fresh(stamp + 0.02))

    def test_long_gap_has_capped_budget_and_never_rebases_on_rejection(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        # Default maximum allowance is 2 + 8 * min(dt, 1) = 10 metres.
        self.assertFalse(self.accept_position(11, 0, stamp + 10))
        self.assertEqual(self.anchor(), anchor)
        self.assertTrue(self.guard.ready)
        self.assertFalse(self.accept_position(1000, 0, stamp + 1000))
        self.assertEqual(self.anchor(), anchor)
        self.assertTrue(self.accept_position(9, 0, stamp + 1000.02))
        self.assertTrue(self.guard.is_fresh(stamp + 1000.02))

    def test_nonfinite_fields_and_out_of_bounds_never_change_anchor(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        for invalid in (float('nan'), float('inf'), -float('inf')):
            for lat, lon, source, now in (
                    (invalid, LONGITUDE, stamp + 0.02, stamp + 0.02),
                    (LATITUDE, invalid, stamp + 0.02, stamp + 0.02),
                    (LATITUDE, LONGITUDE, invalid, stamp + 0.02),
                    (LATITUDE, LONGITUDE, stamp + 0.02, invalid)):
                self.assertFalse(self.guard.accept(lat, lon, source, now))
                self.assertEqual(self.anchor(), anchor)
            self.assertFalse(self.guard.is_fresh(invalid))
        for lat, lon in ((29.9, LONGITUDE), (40.1, LONGITUDE),
                         (LATITUDE, 109.9), (LATITUDE, 125.1)):
            self.assertFalse(self.guard.accept(lat, lon, stamp + 0.02, stamp + 0.02))
            self.assertEqual(self.anchor(), anchor)

    def test_invalid_numeric_parameters_fail_early(self):
        parameters = ('max_speed_mps', 'jump_tolerance_m', 'max_gap_s',
                      'max_age_s', 'future_tolerance_s', 'lat_min', 'lat_max',
                      'lon_min', 'lon_max')
        for name in parameters:
            for value in (float('nan'), float('inf'), -float('inf')):
                self.assertRaises(ValueError, GpsIntegrityGuard, **{name: value})
        for name in ('max_speed_mps', 'jump_tolerance_m', 'max_gap_s',
                     'max_age_s', 'future_tolerance_s'):
            self.assertRaises(ValueError, GpsIntegrityGuard, **{name: -1.0})
        for name in ('max_gap_s', 'max_age_s'):
            self.assertRaises(ValueError, GpsIntegrityGuard, **{name: 0.0})
        self.assertRaises(ValueError, GpsIntegrityGuard, lat_min=40, lat_max=30)
        self.assertRaises(ValueError, GpsIntegrityGuard, lon_min=125, lon_max=110)
        for invalid in (0, -1, 1.5, float('nan'), float('inf')):
            self.assertRaises(ValueError, GpsIntegrityGuard,
                              bootstrap_samples=invalid)


if __name__ == '__main__':
    unittest.main()
