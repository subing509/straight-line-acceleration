"""Offline regression tests: no ROS master or vehicle commands."""
import ast
import math
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace as Obj
import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
LIDAR = ROOT / 'src/lidar/src/lidar_nodes/src'
MAPPER = ROOT / 'src/figure_eight_planner/scripts/jiantu_lidar1.0.3.py'
PLANNER = ROOT / 'src/figure_eight_planner/scripts/bazi_planner_bazi_v4.2.py'


def extract(path, names, namespace):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    body = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
    assert len(body) == len(names)
    exec(compile(ast.Module(body=body, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


class SectorTests(unittest.TestCase):
    def setUp(self):
        self.filter = extract(LIDAR / 'input_and_prefilter.py', ['filter_points'], dict(np=np))['filter_points']

    def cloud(self, points):
        return np.array(points, dtype=[('x', 'f8'), ('y', 'f8'), ('z', 'f8'), ('intensity', 'f8')])

    def test_asymmetric_cones_and_rear_sector(self):
        cloud = self.cloud([(1, 3, .2, 1), (1, -3, .2, 2), (-2, 2, .2, 3),
                            (-2, -2, .2, 4), (-3, 0, .2, 5)])
        selected = self.filter(cloud)
        np.testing.assert_equal(selected['intensity'], [1, 2, 3, 4])

    def test_circle_boundaries_invalid_height_and_organized_cloud(self):
        cloud = self.cloud([(3, 4, .2, 1), (4, 4, .2, 2), (0, 0, .2, 3),
                            (1, 0, .8, 4), (np.nan, 1, .2, 5), (1, 1, np.inf, 6)])
        np.testing.assert_equal(self.filter(cloud.reshape(2, 3))['intensity'], [1])
        self.assertEqual(len(self.filter(cloud[:0])), 0)


class ClusterTests(unittest.TestCase):
    def setUp(self):
        self.cluster = extract(LIDAR / 'clustering.py', ['adaptive_dbscan'],
            dict(np=np, cKDTree=cKDTree, K_NEIGHBORS=5, MIN_PTS=5,
                 epsilon_min=.08, epsilon_max=.25))['adaptive_dbscan']

    def test_two_cones_and_isolated_noise(self):
        shape = np.array([[0, 0], [.01, 0], [0, .01], [-.01, 0], [0, -.01]])
        points = np.vstack((shape+[1, 3], shape+[1, -3], [[4, 0]]))
        labels, epsilon, core = self.cluster(points)
        np.testing.assert_equal(labels[:5], np.zeros(5))
        np.testing.assert_equal(labels[5:10], np.ones(5))
        self.assertEqual(labels[10], -1)
        self.assertTrue(np.all((epsilon >= .08) & (epsilon <= .25)))

    def test_empty_single_and_sparse(self):
        for n in (0, 1, 4):
            labels, _, _ = self.cluster(np.zeros((n, 2)))
            self.assertEqual(len(labels), n)
            self.assertTrue(np.all(labels == -1))


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.ns = extract(MAPPER, ['interpolate_samples', 'CoordinateTransformer', 'ConeObject', 'FSAESLAMNode'],
            dict(np=np, math=math, rospy=Obj(Time=Obj(now=lambda: Obj(to_sec=lambda: self.now)),
                 logwarn_throttle=lambda *args: None)))

    def test_interpolation_angle_wrap_and_missing_pose(self):
        interpolate = self.ns['interpolate_samples']
        np.testing.assert_allclose(interpolate([(9.9, 1, 2), (10.1, 3, 4)], 10, .25), [2, 3])
        self.assertAlmostEqual(interpolate([(9.9, 359), (10.1, 1)], 10, .25, True)[0], 360)
        self.assertIsNone(interpolate([(9, 1)], 10, .25))
        self.assertIsNone(interpolate([(9, 1), (11, 3)], 10, .25))

    def test_confidence_and_periodic_cleanup_without_new_detections(self):
        Cone = self.ns['ConeObject']
        cone = Cone(1, 2, 10, .9)
        self.assertAlmostEqual(cone.confidence, .3)
        cone.update(1, 2, 10.1, .6)
        self.assertLess(cone.confidence, .9)
        cls = self.ns['FSAESLAMNode']
        subject = cls.__new__(cls)
        subject.cone_lock = threading.Lock()
        subject.cone_timeout = 3
        subject.cone_map = [Cone(1, 2, 6), Cone(2, 3, 9), Cone(3, 4, 12)]
        sent = []
        subject.publish_cone_map = lambda: sent.append(len(subject.cone_map))
        subject.map_timer_callback(None)
        self.assertEqual(sent, [1])

    def test_bad_frame_stale_and_unmatched_frames_are_not_mapped(self):
        cls = self.ns['FSAESLAMNode']
        subject = cls.__new__(cls)
        subject.pose_time_tolerance, subject.cone_timeout = .25, 3
        mapped = []
        subject.state_at_time = lambda stamp: None
        subject.update_cone_map_optimized = lambda *args: mapped.append(args)
        header = Obj(frame_id='map', stamp=Obj(to_sec=lambda: 10, is_zero=lambda: False))
        subject.process_cones(header, [(1, 2, 0, .9)])
        header.frame_id = 'velodyne'
        subject.process_cones(header, [(1, 2, 0, .9)])
        header.stamp.to_sec = lambda: 6
        subject.state_at_time = lambda stamp: self.fail('stale frame reached pose lookup')
        subject.process_cones(header, [(1, 2, 0, .9)])
        self.assertEqual(mapped, [])


class PlanningTests(unittest.TestCase):
    def test_nose_offset_roundtrip_and_side_rejection(self):
        mapper_class = extract(MAPPER, ['FSAESLAMNode'], dict(math=math))['FSAESLAMNode']
        mapper = mapper_class.__new__(mapper_class)
        tree = ast.parse(PLANNER.read_text(encoding='utf-8'))
        method = next(n for cls in tree.body if isinstance(cls, ast.ClassDef)
                      for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_in_lidar_planning_region')
        ns = dict(math=math)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(PLANNER), 'exec'), ns)
        subject = Obj(lidar_offset_x=2.11552, lidar_offset_y=-.02212,
                      lidar_yaw_offset_deg=.5, lidar_region_half_width=4, lidar_region_range=5)
        mapper.lidar_offset_x, mapper.lidar_offset_y = subject.lidar_offset_x, subject.lidar_offset_y
        mapper.lidar_yaw_offset_deg, mapper.lidar_y_sign = .5, -1
        state = dict(current_x=0, current_y=0, current_yaw=0, origin_yaw=0)
        for x, y, expected in [(1, 3, True), (1, -3, True), (-1, 3, False), (4, 4, False), (1, 4.2, False)]:
            vx, vy = mapper.transform_lidar_to_world(x, y, 0, state)
            self.assertEqual(ns['_in_lidar_planning_region'](subject, vx, vy), expected)

    def test_missing_pair_grace_speed_cap_and_recovery(self):
        tree = ast.parse(PLANNER.read_text(encoding='utf-8'))
        method = next(n for cls in tree.body if isinstance(cls, ast.ClassDef)
                      for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_limit_speed_for_lidar')
        ns = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(PLANNER), 'exec'), ns)
        pairs = []
        subject = Obj(lidar_region_enabled=True, lidar_missing_since=None,
                      lidar_loss_grace=.5, lidar_loss_speed=1.,
                      _find_local_center_pairs=lambda: pairs)
        limit = ns['_limit_speed_for_lidar']
        self.assertEqual(limit(subject, 3., 10.), 3.)
        self.assertEqual(limit(subject, 3., 10.6), 1.)
        self.assertEqual(limit(subject, .5, 10.7), .5)
        pairs.append({})
        self.assertEqual(limit(subject, 3., 11.), 3.)
        self.assertIsNone(subject.lidar_missing_since)


if __name__ == '__main__':
    unittest.main()
