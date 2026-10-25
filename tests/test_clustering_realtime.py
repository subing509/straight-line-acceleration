"""No ROS or vehicle output: exercise freshness and bounded work under load."""
from __future__ import division

import os
import json
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)),
                              'src', 'lidar', 'src', 'lidar_nodes', 'src'))
from clustering_realtime import LatestFrameWorker


class Clock(object):
    def __init__(self, value=100.0):
        self.value = value

    def __call__(self):
        return self.value


class LatestFrameTests(unittest.TestCase):
    def setUp(self):
        self.workers = []

    def tearDown(self):
        for worker in self.workers:
            worker.stop()

    def worker(self, **kwargs):
        clock = kwargs.pop('clock', Clock())
        worker = LatestFrameWorker(
            source_stamp=lambda message: message['stamp'], now_ros=clock,
            processor=kwargs.pop('processor', lambda msg: dict(id=msg['id'])),
            deliver=kwargs.pop('deliver', lambda result, message: None),
            **kwargs)
        self.workers.append(worker)
        return worker, clock

    def wait(self, predicate, seconds=2.0):
        deadline = time.time() + seconds
        while not predicate() and time.time() < deadline:
            time.sleep(0.002)
        self.assertTrue(predicate(), 'worker did not reach expected state')

    def test_slow_frame_overwrites_intermediate_frames(self):
        entered, release = threading.Event(), threading.Event()
        delivered, processed = [], []

        def process(message):
            processed.append(message['id'])
            if message['id'] == 1:
                entered.set()
                release.wait(2)
            return dict(id=message['id'])

        worker, clock = self.worker(
            processor=process,
            deliver=lambda result, msg: delivered.append(result['id']))
        worker.submit(dict(id=1, stamp=99.9))
        self.assertTrue(entered.wait(1))
        try:
            for index in range(2, 102):
                worker.submit(dict(id=index, stamp=99.9 + index * .0001))
            self.assertTrue(worker.snapshot()['pending'])
        finally:
            release.set()
        self.wait(lambda: len(delivered) == 2)
        self.assertEqual(processed, [1, 101])
        self.assertEqual(delivered, [1, 101])
        self.assertEqual(worker.snapshot()['counts']['replaced'], 99)

    def test_expired_on_receipt_never_computes(self):
        processed = []
        worker, clock = self.worker(processor=lambda msg: processed.append(msg))
        self.assertFalse(worker.submit(dict(id=1, stamp=99.0)))
        self.assertEqual(processed, [])
        self.assertEqual(worker.snapshot()['counts']['drop_before'], 1)

    def test_queued_frame_can_expire_before_computation(self):
        processed = []
        worker, clock = self.worker(auto_start=False,
                                    processor=lambda msg: processed.append(msg))
        self.assertTrue(worker.submit(dict(id=1, stamp=99.9)))
        clock.value = 101.0
        worker.start()
        self.wait(lambda: worker.snapshot()['counts']['drop_before'] == 1)
        self.assertEqual(processed, [])

    def test_expired_result_is_not_delivered(self):
        delivered = []
        clock = Clock()

        def process(message):
            clock.value = 100.5
            return dict(id=message['id'])

        worker, unused = self.worker(clock=clock, processor=process,
                                    deliver=lambda result, msg: delivered.append(msg))
        worker.submit(dict(id=1, stamp=99.9))
        self.wait(lambda: worker.snapshot()['counts']['drop_after'] == 1)
        self.assertEqual(delivered, [])
        self.assertEqual(worker.snapshot()['counts']['processed'], 1)

    def test_source_timestamp_and_message_are_preserved(self):
        delivered = []
        worker, clock = self.worker(deliver=lambda result, msg: delivered.append(msg))
        message = dict(id=1, stamp=99.95)
        worker.submit(message)
        self.wait(lambda: len(delivered) == 1)
        self.assertIs(delivered[0], message)
        self.assertEqual(delivered[0]['stamp'], 99.95)
        self.assertEqual(worker.snapshot()['last']['source_stamp'], 99.95)

    def test_invalid_future_duplicate_and_out_of_order_do_not_refresh(self):
        worker, clock = self.worker(auto_start=False)
        for stamp in (0, float('nan'), float('inf'), 100.2):
            self.assertFalse(worker.submit(dict(id=1, stamp=stamp)))
        self.assertTrue(worker.submit(dict(id=2, stamp=99.95)))
        self.assertFalse(worker.submit(dict(id=3, stamp=99.95)))
        self.assertFalse(worker.submit(dict(id=4, stamp=99.94)))
        self.assertEqual(worker.snapshot()['counts']['invalid'], 6)
        self.assertEqual(worker.snapshot()['counts']['duplicate'], 2)
        self.assertEqual(worker.snapshot()['counts']['replaced'], 0)

    def test_ros_clock_rewind_invalidates_inflight_result_then_recovers(self):
        entered, release = threading.Event(), threading.Event()
        delivered = []

        def process(message):
            if message['id'] == 1:
                entered.set()
                release.wait(2)
            return dict(id=message['id'])

        worker, clock = self.worker(
            processor=process,
            deliver=lambda result, msg: delivered.append(msg['id']))
        worker.submit(dict(id=1, stamp=99.9))
        self.assertTrue(entered.wait(1))
        clock.value = 90.0
        try:
            self.assertFalse(worker.submit(dict(id=2, stamp=89.9)))
            self.assertTrue(worker.submit(dict(id=3, stamp=89.95)))
        finally:
            release.set()
        self.wait(lambda: delivered == [3])
        self.assertEqual(worker.snapshot()['counts']['drop_after'], 1)
        self.assertEqual(worker.snapshot()['counts']['clock_rewinds'], 1)

    def test_duration_timer_rewind_drops_result(self):
        times = iter((10.0, 9.0))
        delivered = []
        worker, clock = self.worker(clock_mono=lambda: next(times),
                                    deliver=lambda result, msg: delivered.append(msg))
        worker.submit(dict(id=1, stamp=99.9))
        self.wait(lambda: worker.snapshot()['counts']['drop_after'] == 1)
        self.assertEqual(delivered, [])
        self.assertEqual(worker.snapshot()['counts']['clock_rewinds'], 1)

    def test_processor_error_does_not_kill_worker(self):
        delivered, errors = [], []

        def process(message):
            if message['id'] == 1:
                raise ValueError('bad frame')
            return dict(id=message['id'])

        worker, clock = self.worker(
            processor=process, on_error=errors.append,
            deliver=lambda result, msg: delivered.append(msg['id']))
        worker.submit(dict(id=1, stamp=99.9))
        self.wait(lambda: worker.snapshot()['counts']['errors'] == 1)
        worker.submit(dict(id=2, stamp=99.95))
        self.wait(lambda: delivered == [2])
        self.assertEqual(len(errors), 1)

    def test_delivery_error_does_not_kill_worker(self):
        delivered = []

        def deliver(result, message):
            if message['id'] == 1:
                raise RuntimeError('publisher failed')
            delivered.append(message['id'])

        worker, clock = self.worker(deliver=deliver)
        worker.submit(dict(id=1, stamp=99.9))
        self.wait(lambda: worker.snapshot()['counts']['errors'] == 1)
        worker.submit(dict(id=2, stamp=99.95))
        self.wait(lambda: delivered == [2])
        self.assertEqual(worker.snapshot()['counts']['published'], 1)

    def test_stop_discards_pending_and_inflight_result(self):
        entered, release = threading.Event(), threading.Event()
        delivered = []

        def process(message):
            entered.set()
            release.wait(2)
            return dict(id=message['id'])

        worker, clock = self.worker(processor=process,
                                    deliver=lambda result, msg: delivered.append(msg))
        worker.submit(dict(id=1, stamp=99.9))
        self.assertTrue(entered.wait(1))
        worker.submit(dict(id=2, stamp=99.95))
        self.assertFalse(worker.stop(join_timeout=0.01))
        self.assertFalse(worker.submit(dict(id=3, stamp=99.96)))
        release.set()
        self.assertTrue(worker.stop())
        self.assertEqual(delivered, [])
        self.assertEqual(worker.snapshot()['counts']['drop_before'], 1)
        self.assertEqual(worker.snapshot()['counts']['drop_after'], 1)

    def test_status_callback_failure_is_reported_and_worker_survives(self):
        delivered = []

        def status(snapshot):
            raise ValueError('status failed')

        worker, clock = self.worker(on_status=status,
                                    deliver=lambda result, msg: delivered.append(msg))
        worker.submit(dict(id=1, stamp=99.9))
        self.wait(lambda: worker.snapshot()['counts']['errors'] == 1)
        worker.submit(dict(id=2, stamp=99.95))
        self.wait(lambda: len(delivered) == 2)

    def test_snapshot_sanitizes_nonfinite_metrics(self):
        worker, clock = self.worker(processor=lambda msg: dict(
            stage_s=dict(bad=float('nan'), inf=float('inf'), good=0.001),
            input_points=12, filtered_points=10))
        worker.submit(dict(id=1, stamp=99.9))
        self.wait(lambda: worker.snapshot()['counts']['published'] == 1)
        snapshot = worker.snapshot()
        self.assertIsNone(snapshot['last']['stage_s']['bad'])
        self.assertIsNone(snapshot['last']['stage_s']['inf'])
        self.assertEqual(snapshot['last']['stage_s']['good'], 0.001)
        json.dumps(snapshot, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
