# -*- coding: utf-8 -*-
"""Heading continuity tests using only the standard library."""
from __future__ import print_function

import os
import sys
import unittest


MODULE_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                          'src', 'acceleration_event', 'src')
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)
from heading_integrity import HeadingIntegrityGuard


class HeadingIntegrityTest(unittest.TestCase):
    def setUp(self):
        self.guard = HeadingIntegrityGuard()

    def bootstrap(self, heading=0.0, start=100.0):
        for index in range(2):
            self.assertFalse(self.guard.accept(heading, start+index*0.02,
                                              start+index*0.02))
            self.assertFalse(self.guard.ready)
        self.assertTrue(self.guard.accept(heading, start+0.04, start+0.04))
        self.assertTrue(self.guard.ready)
        return start+0.04

    def anchor(self):
        return self.guard.heading, self.guard.stamp

    def test_stationary_three_sample_startup(self):
        self.assertIsNone(self.guard.heading)
        self.assertIsNone(self.guard.stamp)
        self.assertFalse(self.guard.is_fresh(100.0))
        stamp = self.bootstrap(40.0)
        for index in range(1, 101):
            now = stamp+index*0.02
            self.assertTrue(self.guard.accept(40.0, now, now))
        self.assertEqual(self.guard.heading, 40.0)
        self.assertEqual(self.guard.counts['received'], 103)

    def test_normal_continuous_turning_and_original_heading_values(self):
        stamp = self.bootstrap(0.0)
        for index in range(1, 501):
            now, heading = stamp+index*0.02, index*2.0
            self.assertTrue(self.guard.accept(heading, now, now))
            self.assertEqual(self.guard.heading, heading)
        self.assertEqual(self.guard.heading, 1000.0)

    def test_wraparound_180_0_and_360_is_continuous(self):
        for sequence in ((179.0, -179.0, 179.0), (-1.0, 1.0, 359.0, 361.0),
                         (359.0, 1.0, 360.0, 0.0, 720.0, -360.0)):
            self.guard = HeadingIntegrityGuard()
            stamp = self.bootstrap(sequence[0])
            for index, heading in enumerate(sequence[1:], 1):
                now = stamp+index*0.02
                self.assertTrue(self.guard.accept(heading, now, now))
                self.assertEqual(self.guard.heading, heading)

    def test_bad_first_heading_can_reseed_from_three_normal_samples(self):
        self.assertFalse(self.guard.accept(157.0, 100.0, 100.0))
        for stamp in (100.02, 100.04):
            self.assertFalse(self.guard.accept(0.0, stamp, stamp))
            self.assertFalse(self.guard.ready)
        self.assertTrue(self.guard.accept(0.0, 100.06, 100.06))
        self.assertEqual(self.guard.heading, 0.0)

    def test_single_impossible_spike_keeps_anchor_then_recovers(self):
        stamp = self.bootstrap(45.0)
        anchor = self.anchor()
        self.assertFalse(self.guard.accept(-112.1, stamp+0.02, stamp+0.02))
        self.assertEqual(self.anchor(), anchor)
        self.assertEqual(self.guard.last_reason, 'jump')
        self.assertAlmostEqual(self.guard.last_step_deg, 157.1)
        self.assertAlmostEqual(self.guard.last_limit_deg, 8.6)
        self.assertTrue(self.guard.accept(46.0, stamp+0.04, stamp+0.04))
        self.assertEqual(self.guard.heading, 46.0)

    def test_continuous_time_rejections_do_not_refresh_or_clear_ready(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        for index in range(1, 51):
            now = stamp+index*0.02
            # Data remains delayed: no valid measurement is being refreshed.
            self.assertFalse(self.guard.accept(157.0, stamp-1.0, now))
        self.assertEqual(self.anchor(), anchor)
        self.assertTrue(self.guard.ready)
        self.assertFalse(self.guard.is_fresh(stamp+1.0))
        self.assertTrue(self.guard.accept(1.0, stamp+1.02, stamp+1.02))

    def test_rejected_fast_spikes_do_not_advance_the_anchor(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        # Once enough time has elapsed a large real turn becomes possible.
        # Use a short burst to test only physically impossible transitions.
        for index in range(1, 42):
            now = stamp+index*0.02
            self.assertFalse(self.guard.accept(157.0, now, now))
            self.assertEqual(self.anchor(), anchor)
        self.assertFalse(self.guard.is_fresh(stamp+0.82))
        self.assertTrue(self.guard.accept(0.0, stamp+0.84, stamp+0.84))

    def test_duplicate_out_of_order_zero_delayed_and_future_stamps(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        cases = ((stamp, stamp+0.02), (stamp-0.01, stamp+0.02),
                 (0.0, stamp+0.02), (-1.0, stamp+0.02),
                 (stamp+0.02, stamp+0.62), (stamp+1.0, stamp+0.02))
        for source, receipt in cases:
            self.assertFalse(self.guard.accept(1.0, source, receipt))
            self.assertEqual(self.anchor(), anchor)
        self.assertTrue(self.guard.accept(1.0, stamp+0.04, stamp+0.04))

    def test_clock_rewind_and_freshness_use_last_accepted_stamp(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        self.assertTrue(self.guard.is_fresh(stamp+0.49))
        self.assertFalse(self.guard.is_fresh(stamp+0.51))
        self.assertFalse(self.guard.is_fresh(stamp-0.5))
        self.assertFalse(self.guard.accept(1.0, stamp-0.5, stamp-0.5))
        self.assertEqual(self.anchor(), anchor)
        self.assertTrue(self.guard.ready)
        self.assertTrue(self.guard.accept(1.0, stamp+0.02, stamp+0.02))

    def test_gap_budget_is_capped_with_custom_rate(self):
        self.guard = HeadingIntegrityGuard(max_rate_dps=20.0)
        stamp = self.bootstrap()
        anchor = self.anchor()
        self.assertFalse(self.guard.accept(30.0, stamp+100, stamp+100))
        self.assertEqual(self.anchor(), anchor)
        self.assertEqual(self.guard.last_limit_deg, 25.0)
        self.assertTrue(self.guard.accept(24.0, stamp+100.02, stamp+100.02))

    def test_bootstrap_needs_recent_consistent_samples(self):
        self.guard.accept(0, 100.0, 100.0)
        self.guard.accept(0, 100.02, 100.02)
        self.assertFalse(self.guard.accept(0, 102.0, 102.0))
        self.assertFalse(self.guard.ready)
        self.assertFalse(self.guard.accept(0, 102.02, 102.02))
        self.assertTrue(self.guard.accept(0, 102.04, 102.04))

    def test_nonfinite_fields_are_rejected_without_changing_anchor(self):
        stamp = self.bootstrap()
        anchor = self.anchor()
        for invalid in (float('nan'), float('inf'), -float('inf'), None, True):
            for heading, source, now in ((invalid, stamp+0.02, stamp+0.02),
                                         (0.0, invalid, stamp+0.02),
                                         (0.0, stamp+0.02, invalid)):
                self.assertFalse(self.guard.accept(heading, source, now))
                self.assertEqual(self.anchor(), anchor)
            self.assertFalse(self.guard.is_fresh(invalid))

    def test_snapshot_reports_accepted_anchor_and_copies_counts(self):
        stamp = self.bootstrap(30)
        self.guard.accept(180, stamp+0.02, stamp+0.02)
        state = self.guard.snapshot(stamp+0.03)
        self.assertTrue(state['ready'])
        self.assertTrue(state['fresh'])
        self.assertEqual(state['accepted_heading'], 30)
        self.assertEqual(state['accepted_stamp'], stamp)
        self.assertAlmostEqual(state['accepted_age_s'], 0.03)
        self.assertEqual(state['last_reason'], 'jump')
        self.assertEqual(state['last_step_deg'], 150)
        self.assertAlmostEqual(state['last_limit_deg'], 8.6)
        state['counts']['accepted'] = -1
        self.assertEqual(self.guard.counts['accepted'], 1)

    def test_invalid_parameters_fail_early(self):
        for name in ('max_rate_dps', 'jump_tolerance_deg', 'max_gap_s',
                     'max_age_s', 'future_tolerance_s'):
            for invalid in (float('nan'), float('inf'), -float('inf'), -1, None, True):
                self.assertRaises(ValueError, HeadingIntegrityGuard, **{name: invalid})
        for name in ('max_rate_dps', 'jump_tolerance_deg', 'max_gap_s', 'max_age_s'):
            self.assertRaises(ValueError, HeadingIntegrityGuard, **{name: 0})
        for invalid in (0, 1, -1, 1.5, float('nan'), float('inf'), True):
            self.assertRaises(ValueError, HeadingIntegrityGuard, bootstrap_samples=invalid)


if __name__ == '__main__':
    unittest.main()
