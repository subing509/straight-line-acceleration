#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Bounded latest-frame processing, without ROS or vehicle dependencies.

``processor`` computes a result only. ``deliver`` publishes that result after
the worker has checked source age again. An overwritten frame is intentionally
discarded; this class never builds a FIFO of point clouds.
"""
from __future__ import division

import math
import numbers
import threading
import timeit

try:
    _string_types = (str, unicode)
except NameError:
    _string_types = (str,)


def _finite(value):
    return not (math.isnan(value) or math.isinf(value))


def _json_safe(value):
    """Copy metrics into finite JSON primitives, including NumPy scalars."""
    if value is None or isinstance(value, (bool,) + _string_types):
        return value
    if isinstance(value, numbers.Integral):
        return int(value)
    if isinstance(value, numbers.Real):
        number = float(value)
        return number if _finite(number) else None
    if isinstance(value, dict):
        return dict((str(key), _json_safe(item)) for key, item in value.items())
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    return None


class LatestFrameWorker(object):
    """Run one computation at a time, retaining at most one pending frame.

    Times returned by ``source_stamp`` and ``now_ros`` must be seconds in the
    same clock domain. Results keep their original message; no timestamp is
    rewritten. ``clock_mono`` measures durations only and defaults to
    ``timeit.default_timer`` for Python 2 compatibility. A negative measured
    duration is rejected because that timer can use wall time on Python 2.

    ``on_status(snapshot)`` runs after an attempted computation, outside the
    input lock. ``on_error(exception)`` reports failures without killing the
    worker. Delivery is serialized with stop, so after stop returns no later
    result can be delivered, even if an in-flight computation is still running.
    """

    def __init__(self, source_stamp, now_ros, processor, deliver,
                 on_status=None, on_error=None, max_age=0.4,
                 future_tolerance=0.1, clock_mono=None, auto_start=True):
        self.source_stamp = source_stamp
        self.now_ros = now_ros
        self.processor = processor
        self.deliver = deliver
        self.on_status = on_status
        self.on_error = on_error
        self.max_age = float(max_age)
        self.future_tolerance = float(future_tolerance)
        if not _finite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_age must be finite and positive')
        if not _finite(self.future_tolerance) or self.future_tolerance < 0:
            raise ValueError('future_tolerance must be finite and nonnegative')
        self.clock_mono = clock_mono or timeit.default_timer
        self._condition = threading.Condition(threading.RLock())
        self._delivery_lock = threading.RLock()
        self._pending = None
        self._thread = None
        self._stopped = False
        self._processing = False
        self._last_stamp = None
        self._last_ros_now = None
        self._generation = 0
        self._counts = dict((key, 0) for key in (
            'received', 'replaced', 'drop_before', 'drop_after', 'invalid',
            'errors', 'processed', 'published', 'clock_rewinds', 'duplicate'))
        self._last = {}
        if auto_start:
            self.start()

    def start(self):
        with self._condition:
            if self._stopped:
                return False
            if self._thread is not None:
                return True
            self._thread = threading.Thread(target=self._run,
                                            name='clustering-latest-frame')
            self._thread.daemon = True
            self._thread.start()
            return True

    def _observe_clock_locked(self, now):
        """Invalidate in-flight/queued work on a source-clock epoch change."""
        rewound = (self._last_ros_now is not None and
                   now < self._last_ros_now - self.future_tolerance)
        if rewound:
            self._generation += 1
            self._counts['clock_rewinds'] += 1
            self._last_stamp = None
            if self._pending is not None:
                self._counts['drop_before'] += 1
                self._pending = None
        self._last_ros_now = now
        return rewound

    def submit(self, message):
        """Accept a fresh, advancing observation, replacing any pending one."""
        with self._condition:
            self._counts['received'] += 1
            if self._stopped:
                return False
        try:
            stamp = float(self.source_stamp(message))
            return self._accept(message, stamp)
        except Exception as error:
            with self._condition:
                self._counts['invalid'] += 1
            self._report_error(error, count=False)
            return False

    def _accept(self, message, stamp):
        with self._condition:
            if self._stopped:
                return False
            # Serialize clock reads with freshness checks. A callback that was
            # descheduled must not look like the clock went backwards merely
            # because a worker sampled a newer time before it obtained the lock.
            now = float(self.now_ros())
            if not (_finite(stamp) and stamp > 0 and _finite(now)):
                self._counts['invalid'] += 1
                return False
            if self._observe_clock_locked(now):
                self._counts['invalid'] += 1
                return False
            age = now - stamp
            if age < -self.future_tolerance:
                self._counts['invalid'] += 1
                return False
            if age > self.max_age:
                self._counts['drop_before'] += 1
                return False
            if self._last_stamp is not None and stamp <= self._last_stamp:
                self._counts['invalid'] += 1
                self._counts['duplicate'] += 1
                return False
            self._last_stamp = stamp
            if self._pending is not None:
                self._counts['replaced'] += 1
            self._pending = (message, stamp, self._generation)
            self._condition.notify()
            return True

    def snapshot(self):
        with self._condition:
            last = dict(self._last)
            if isinstance(last.get('stage_s'), dict):
                last['stage_s'] = dict(last['stage_s'])
            return _json_safe(dict(counts=dict(self._counts), last=last,
                                   pending=self._pending is not None,
                                   processing=self._processing,
                                   stopped=self._stopped))

    def stop(self, join_timeout=2.0):
        """Discard pending work and prevent all subsequent delivery.

        A false return only means computation has not finished before the join
        timeout. That computation is still prohibited from publishing results.
        """
        with self._delivery_lock:
            with self._condition:
                self._stopped = True
                if self._pending is not None:
                    self._counts['drop_before'] += 1
                self._pending = None
                if hasattr(self._condition, 'notify_all'):
                    self._condition.notify_all()
                else:
                    self._condition.notifyAll()  # Python 2.7
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(join_timeout)
        return thread is None or not thread.is_alive()

    def _report_error(self, error, count=True):
        if count:
            with self._condition:
                self._counts['errors'] += 1
        if self.on_error is not None:
            try:
                self.on_error(error)
            except Exception:
                # Reporting an error must never terminate the worker.
                pass

    def _status(self):
        if self.on_status is not None:
            try:
                self.on_status(self.snapshot())
            except Exception as error:
                self._report_error(error)

    def _fresh(self, stamp, generation):
        with self._condition:
            now = float(self.now_ros())
            if not _finite(now):
                return False, None
            rewound = self._observe_clock_locked(now)
            age = now - stamp
            return (not self._stopped and not rewound and
                    generation == self._generation and
                    -self.future_tolerance <= age <= self.max_age), age

    def _attempt(self, message, stamp, generation):
        fresh, age = self._fresh(stamp, generation)
        with self._condition:
            self._last = dict(source_stamp=stamp, age_on_start_s=age)
            if not fresh:
                self._counts['drop_before'] += 1
                return
        started = float(self.clock_mono())
        result = self.processor(message)
        elapsed = float(self.clock_mono()) - started
        if not isinstance(result, dict):
            raise TypeError('processor must return a dict')
        with self._condition:
            self._counts['processed'] += 1
            self._last['processing_s'] = elapsed if _finite(elapsed) else None
            for key in ('input_points', 'filtered_points'):
                if key in result:
                    self._last[key] = result[key]
            if isinstance(result.get('stage_s'), dict):
                self._last['stage_s'] = dict(result['stage_s'])
        with self._delivery_lock:
            fresh, age = self._fresh(stamp, generation)
            with self._condition:
                self._last['age_on_publish_s'] = age
                if not _finite(elapsed) or elapsed < 0:
                    self._counts['clock_rewinds'] += 1
                    fresh = False
                if not fresh:
                    self._counts['drop_after'] += 1
                    return
            self.deliver(result, message)
            with self._condition:
                self._counts['published'] += 1

    def _run(self):
        while True:
            with self._condition:
                while self._pending is None and not self._stopped:
                    self._condition.wait()
                if self._stopped:
                    return
                message, stamp, generation = self._pending
                self._pending = None
                self._processing = True
            try:
                self._attempt(message, stamp, generation)
            except Exception as error:
                self._report_error(error)
            finally:
                with self._condition:
                    self._processing = False
                self._status()
