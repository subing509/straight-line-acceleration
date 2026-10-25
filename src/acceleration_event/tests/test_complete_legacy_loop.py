#!/usr/bin/env python3
"""Full production control loop through the explicitly assumed offline plant."""
import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('legacy_loop_validation',
    Path(__file__).resolve().parents[1]/'scripts'/'legacy_loop_validation.py')
loop=importlib.util.module_from_spec(spec)
spec.loader.exec_module(loop)


class CompleteLoopTests(unittest.TestCase):
    def assert_three_frame_imu_start(self, records):
        # The first two fresh, consistent IMU frames are warming up. A zero
        # placeholder command alone does not prove the controller executed.
        self.assertGreaterEqual(len(records), 3)
        for record in records[:2]:
            self.assertEqual('waiting', record['state'])
            self.assertIsNone(record['stop_reason'])
            self.assertEqual(0.0, record['command'])
            self.assertEqual(0, record['control_records'])
            self.assertIsNone(record['filtered_lateral'])
            self.assertIsNone(record['filtered_heading'])
        self.assertEqual('running', records[2]['state'])
        self.assertEqual(1, records[2]['control_records'])
        self.assertIsNotNone(records[2]['filtered_lateral'])
        self.assertIsNotNone(records[2]['filtered_heading'])

    def test_stationary_start_executes_real_control_every_tick(self):
        summary,records=loop.run_case('stationary',speed=0.,duration=3.)
        self.assert_three_frame_imu_start(records)
        self.assertTrue(summary['started'])
        self.assertIsNone(summary['stop_reason'])
        self.assertEqual(60,len(records))
        self.assertEqual(58,summary['full_control_records'])
        self.assertEqual(list(range(1,59)),[r['control_records'] for r in records[2:]])
        self.assertEqual([],summary['errors'])
        self.assertAlmostEqual(.6,summary['final_y'])

    def test_simple_offset_cases_are_symmetric_under_assumed_plant(self):
        right,rr=loop.run_case('right')
        left,lr=loop.run_case('left',initial_y=-.6)
        self.assert_three_frame_imu_start(rr)
        self.assert_three_frame_imu_start(lr)
        first_right=next(r for r in rr if r['control_records']>0)
        first_left=next(r for r in lr if r['control_records']>0)
        self.assertLess(first_right['command'],0.)
        self.assertGreater(first_left['command'],0.)
        self.assertLess(abs(right['final_y']),.1)
        self.assertLess(abs(left['final_y']),.1)
        self.assertAlmostEqual(right['final_y'],-left['final_y'],places=8)
        self.assertEqual(238,right['full_control_records'])
        self.assertEqual(238,left['full_control_records'])

    def test_one_row_continuous_control_does_not_require_second_row(self):
        summary,_=loop.run_case('one_row',rows=1)
        self.assertTrue(summary['started'])
        self.assertIsNone(summary['stop_reason'])
        self.assertLess(abs(summary['final_y']),.1)

    def test_short_road_gap_recovers_without_latch(self):
        summary,records=loop.run_case('short_gap',drop=('road',1.,1.2),duration=4.)
        self.assert_three_frame_imu_start(records)
        self.assertIsNone(summary['stop_reason'])
        self.assertEqual(80,len(records))
        self.assertEqual(78,summary['full_control_records'])
        self.assertEqual(list(range(1,79)),[r['control_records'] for r in records[2:]])

    def test_long_gaps_latch_stop_and_do_not_resume_on_return(self):
        for source in ('road','pose','imu'):
            summary,records=loop.run_case('loss_'+source,drop=(source,1.,2.5),duration=4.)
            self.assertEqual(source+'_stale',summary['stop_reason'])
            stopped=[r for r in records if r['state']=='stopped']
            self.assertTrue(stopped)
            self.assertEqual('stopped',records[-1]['state'])
            self.assertTrue(all(r['command']==0. for r in stopped))
            self.assertEqual([],summary['errors'])


if __name__=='__main__':
    unittest.main()
