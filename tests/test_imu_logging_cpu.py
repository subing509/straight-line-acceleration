# -*- coding: utf-8 -*-
"""Exercise IMU logging without importing ROS or opening a serial device."""
from __future__ import print_function

import ast
import copy
import math
import os
import struct
import unittest


SOURCE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                           'src', 'imu', 'src', 'daoyuan_simple.py')


def load_functions(namespace, names=('FrameLog',)):
    with open(SOURCE_PATH, 'rb') as source_file:
        tree = ast.parse(source_file.read(), filename=SOURCE_PATH)
    fields = {'body': [node for node in tree.body
                       if isinstance(node, (ast.ClassDef, ast.FunctionDef))
                       and node.name in names]}
    if 'type_ignores' in ast.Module._fields:
        fields['type_ignores'] = []
    module = ast.Module(**fields)
    eval(compile(module, SOURCE_PATH, 'exec'), namespace)
    return namespace


class Clock(object):
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


class Stream(object):
    def __init__(self):
        self.lines = []
        self.persisted = []
        self.flush_times = []
        self.closed = False
        self.clock = None
        self.fail_flush = False

    def write(self, line):
        self.lines.append(line)

    def flush(self):
        self.flush_times.append(self.clock() if self.clock else None)
        if self.fail_flush:
            raise IOError('flush failed')
        self.persisted = list(self.lines)

    def close(self):
        self.closed = True


class FrameLogTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.stream = Stream()
        self.stream.clock = self.clock
        self.logs = []
        namespace = load_functions({'math': math, 'default_timer': self.clock})
        self.log_class = namespace['FrameLog']

    def make_log(self, **kwargs):
        return self.log_class(self.stream, lambda *args: self.logs.append(args),
                              clock=self.clock, **kwargs)

    def test_every_record_is_preserved_and_summary_is_throttled(self):
        log = self.make_log()
        for number in range(100):
            self.clock.now = number / 100.0
            log.record(number + 1, 39.1, 116.2, number * 0.5, 12.3)
        expected = ['39.1000000\t116.2000000\t%.2f\n' % (n * 0.5)
                    for n in range(100)]
        self.assertEqual(self.stream.lines, expected)
        self.assertEqual(len(self.logs), 1)
        self.assertEqual(self.stream.flush_times, [])
        self.clock.now = 1.0
        log.tick()
        self.assertEqual(self.stream.persisted, expected)
        self.assertEqual(self.stream.flush_times, [1.0])
        self.assertEqual(len(self.logs), 2)
        self.assertEqual(self.logs[-1][1:3], (100, 99))

    def test_idle_ticks_flush_once_and_do_not_repeat_empty_summaries(self):
        log = self.make_log()
        log.record(1, 1, 2, 3, 4)
        self.clock.now = 1.0
        log.tick()
        self.clock.now = 2.0
        log.tick()
        self.assertEqual(self.stream.flush_times, [1.0])
        self.assertEqual(len(self.logs), 1)

    def test_intervals_are_independent_and_flush_is_bounded(self):
        log = self.make_log(log_interval=2.0, flush_interval=0.5)
        for number in range(21):
            self.clock.now = number / 10.0
            log.record(number + 1, 1, 2, 3, 4)
        self.assertEqual(len(self.stream.lines), 21)
        self.assertEqual(self.stream.flush_times, [0.5, 1.0, 1.5, 2.0])
        self.assertEqual(len(self.logs), 2)
        self.assertEqual(self.stream.persisted, self.stream.lines)

    def test_close_persists_pending_records_and_is_idempotent(self):
        log = self.make_log()
        log.record(1, 1, 2, 3, 4)
        log.close()
        log.close()
        self.assertTrue(self.stream.closed)
        self.assertEqual(self.stream.persisted, self.stream.lines)
        self.assertEqual(len(self.stream.flush_times), 1)
        self.assertRaises(ValueError, log.record, 2, 1, 2, 3, 4)

    def test_clock_rewind_flushes_pending_then_resumes_intervals(self):
        self.clock.now = 10.0
        log = self.make_log()
        log.record(1, 1, 2, 3, 4)
        self.clock.now = 10.2
        log.record(2, 1, 2, 3, 4)
        self.clock.now = 5.0
        log.tick()
        self.assertEqual(self.stream.flush_times, [5.0])
        self.assertEqual(self.stream.persisted, self.stream.lines)
        self.assertEqual(len(self.logs), 2)
        self.clock.now = 5.2
        log.record(3, 1, 2, 3, 4)
        self.clock.now = 5.9
        log.tick()
        self.assertEqual(len(self.logs), 2)
        self.assertEqual(len(self.stream.flush_times), 1)
        self.clock.now = 6.0
        log.tick()
        self.assertEqual(len(self.logs), 3)
        self.assertEqual(self.stream.flush_times, [5.0, 6.0])

    def test_close_still_closes_stream_when_flush_fails(self):
        log = self.make_log()
        self.stream.fail_flush = True
        self.assertRaises(IOError, log.close)
        self.assertTrue(self.stream.closed)

    def test_invalid_intervals_fail_before_the_loop(self):
        for invalid in (0, -1, float('nan'), float('inf')):
            self.assertRaises(ValueError, self.make_log, log_interval=invalid)
            self.assertRaises(ValueError, self.make_log, flush_interval=invalid)


class Box(object):
    pass


class Message(object):
    def __init__(self):
        self.header = Box()
        self.status = Box()
        self.orientation = Box()


class Port(object):
    def __init__(self, payload=''):
        self.payload = payload
        self.closed = False
        self.read_sizes = []

    @property
    def in_waiting(self):
        return len(self.payload)

    def read(self, size):
        self.read_sizes.append(size)
        data, self.payload = self.payload[:size], self.payload[size:]
        return data

    def close(self):
        self.closed = True


class Publisher(object):
    def __init__(self, name, publication_order):
        self.name = name
        self.messages = []
        self.publication_order = publication_order

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))
        self.publication_order.append(self.name)


class FakeROS(object):
    def __init__(self, port):
        self.port = port
        self.publishers = {}
        self.publication_order = []
        self.info = []
        self.errors = []
        self.params = []
        self.interrupt = False
        self.Time = Box()
        self.Time.now = lambda: 123

    def init_node(self, *args, **kwargs):
        pass

    def Publisher(self, name, *args, **kwargs):
        publisher = Publisher(name, self.publication_order)
        self.publishers[name] = publisher
        return publisher

    def get_param(self, name, default):
        self.params.append((name, default))
        return default

    def loginfo(self, *args):
        self.info.append(args)

    def logerr(self, *args):
        self.errors.append(args)

    def is_shutdown(self):
        if self.interrupt:
            raise KeyboardInterrupt()
        return not self.port.payload

    def sleep(self, duration):
        raise AssertionError('The data path must not sleep or limit publishing')


def packet(heading_raw, lat_raw, lon_raw, altitude_raw):
    data = bytearray(64)
    data[0:3] = bytearray([0xbd, 0xdb, 0x0b])
    data[7:9] = bytearray(struct.pack('<h', heading_raw))
    data[21:25] = bytearray(struct.pack('<i', lat_raw))
    data[25:29] = bytearray(struct.pack('<i', lon_raw))
    data[29:33] = bytearray(struct.pack('<i', altitude_raw))
    # Give the unchanged ord parser Python 2 serial-string semantics on both
    # runtimes. Real serial byte handling is deliberately outside this test.
    return ''.join(chr(value) for value in data)


class MainCleanupTest(unittest.TestCase):
    def make_main(self, payload='', fail_open=False):
        self.port = Port(payload)
        self.ros = FakeROS(self.port)
        self.stream = Stream()
        serial = Box()
        serial.Serial = lambda **kwargs: self.port
        serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE = 8, 'N', 1
        path = Box()
        path.exists = lambda name: True
        os_stub = Box()
        os_stub.path = path

        def open_log(*args):
            if fail_open:
                raise IOError('cannot open log')
            return self.stream

        namespace = load_functions(
            {'math': math, 'default_timer': Clock(), 'rospy': self.ros,
             'serial': serial, 'Imu': Message, 'NavSatFix': Message,
             'os': os_stub, 'open': open_log}, ('FrameLog', 'main'))
        return namespace['main']

    def test_two_frames_publish_and_write_each_original_record(self):
        main = self.make_main(packet(8192, 391000000, 1162000000, 12300) +
                              packet(-8192, -391000000, -1162000000, -12300))
        self.assertEqual(main(), 0)
        self.assertEqual(self.port.read_sizes, [128])
        self.assertEqual(self.ros.publication_order,
                         ['imu_data', 'GPS_data', 'imu_data', 'GPS_data'])
        self.assertEqual([m.orientation.z for m in
                          self.ros.publishers['imu_data'].messages], [90.0, -90.0])
        self.assertEqual([m.latitude for m in
                          self.ros.publishers['GPS_data'].messages], [39.1, -39.1])
        self.assertEqual(self.stream.lines, ['39.1000000\t116.2000000\t90.00\n',
                                            '-39.1000000\t-116.2000000\t-90.00\n'])
        self.assertEqual(self.stream.persisted, self.stream.lines)
        self.assertEqual(len(self.stream.flush_times), 1)
        self.assertTrue(self.stream.closed)
        self.assertTrue(self.port.closed)
        self.assertEqual(self.ros.errors, [])
        self.assertEqual(self.ros.params,
                         [('~log_interval', 1.0), ('~log_flush_interval', 1.0)])
        summaries = [args for args in self.ros.info
                     if args[0].startswith('IMU/GPS packets=')]
        self.assertEqual(len(summaries), 1)

    def test_interrupt_closes_log_and_serial(self):
        main = self.make_main()
        self.ros.interrupt = True
        self.assertRaises(KeyboardInterrupt, main)
        self.assertTrue(self.stream.closed)
        self.assertEqual(len(self.stream.flush_times), 1)
        self.assertTrue(self.port.closed)

    def test_file_open_failure_still_closes_serial(self):
        main = self.make_main(fail_open=True)
        self.assertRaises(IOError, main)
        self.assertTrue(self.port.closed)

    def test_final_flush_failure_still_closes_file_and_serial(self):
        main = self.make_main()
        self.stream.fail_flush = True
        self.assertRaises(IOError, main)
        self.assertTrue(self.stream.closed)
        self.assertTrue(self.port.closed)


if __name__ == '__main__':
    unittest.main()
