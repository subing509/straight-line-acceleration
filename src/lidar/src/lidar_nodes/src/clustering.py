#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from timeit import default_timer
import json
import math
from std_msgs.msg import String
from clustering_realtime import LatestFrameWorker
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import ros_numpy as rnp
import numpy as np
import uuid
from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
import random


# ============================================================
# 自适应DBSCAN参数
# ============================================================

# 公式(10)中的k近邻数量
K_NEIGHBORS = 5

# 公式(9)中的MinPts
MIN_PTS = 5


# ============================================================
# LiDAR置信度模型参数
# ============================================================

# 点数量范围
N_MIN = 3
N_MAX = 5

# 锥桶高度范围，单位：m
H_MIN = 0.15
H_MAX = 0.40

# XY平面等效半径范围，单位：m
R_MIN = 0.05
R_MAX = 0.30

# 最大有效检测距离，单位：m
D_MAX = 30.0

# 各特征权重
W_N = 0.6
W_H = 0
W_R = 0
W_D = 0.4

# 与 jiantu.py 使用相同的车辆坐标系过滤范围（X 前，Y 右），单位米。
LIDAR_OFFSET_X = 1.76618
LIDAR_OFFSET_Y = -0.11304
LIDAR_Y_SIGN = -1.0
LIDAR_YAW_OFFSET_DEG = 0.5
SELF_FILTER_REAR = 1.2
SELF_FILTER_FRONT = 2.7
SELF_FILTER_HALF_WIDTH = 0.9
VISUALIZATION_LIFETIME = 0.1


# ============================================================
# 原始PoseArray代码
# ============================================================

def get_PoseArray(idxs_by_cluster,
                  nparray_obstacle_xy,
                  starray_obstacle_xyz,
                  timestamp):

    clusters_keys = idxs_by_cluster.keys()

    pose_array = PoseArray()
    pose_array.header.stamp = timestamp
    pose_array.header.frame_id = "/velodyne"

    for _, idx in enumerate(clusters_keys):

        pose = Pose()

        # Cluster centroid position
        cluster_indices = idxs_by_cluster[idx]

        pose.position.x = np.mean(
            nparray_obstacle_xy[cluster_indices, 0]
        )

        pose.position.y = np.mean(
            nparray_obstacle_xy[cluster_indices, 1]
        )

        pose.position.z = np.mean(
            starray_obstacle_xyz['z'][cluster_indices]
        )

        pose_array.poses.append(pose)

    return pose_array


# ============================================================
# RVIZ Marker
# ============================================================

def get_cone_marker(timestamp, id, x, y, z, r, g, b, a):

    lidar_marker = Marker()

    lidar_marker.header.frame_id = "/velodyne"
    lidar_marker.header.stamp = timestamp

    lidar_marker.ns = "aux_marker"
    lidar_marker.action = Marker.ADD
    lidar_marker.id = id
    lidar_marker.type = lidar_marker.CYLINDER

    lidar_marker.scale.x = 0.3
    lidar_marker.scale.y = 0.3
    lidar_marker.scale.z = 0.3

    lidar_marker.color.a = a
    lidar_marker.color.r = r
    lidar_marker.color.g = g
    lidar_marker.color.b = b

    lidar_marker.pose.orientation.w = 1.0

    lidar_marker.pose.position.x = x
    lidar_marker.pose.position.y = y
    lidar_marker.pose.position.z = z

    return lidar_marker


def get_LiDAR_marker(timestamp):

    lidar_marker = Marker()

    lidar_marker.header.frame_id = "/velodyne"
    lidar_marker.header.stamp = timestamp

    lidar_marker.ns = "cone_markers"
    lidar_marker.action = Marker.ADD
    lidar_marker.id = 250
    lidar_marker.type = lidar_marker.CYLINDER

    lidar_marker.scale.x = 0.3
    lidar_marker.scale.y = 0.3
    lidar_marker.scale.z = 0.3

    lidar_marker.color.a = 1
    lidar_marker.color.r = 0
    lidar_marker.color.g = 1
    lidar_marker.color.b = 1

    lidar_marker.pose.orientation.w = 1.0

    lidar_marker.pose.position.x = 0
    lidar_marker.pose.position.y = 0
    lidar_marker.pose.position.z = 0

    return lidar_marker


def publish_markers_to_RVIZ(
        idxs_by_cluster,
        nparray_obstacle_xy,
        starray_obstacle_xyz, timestamp=None):

    clusters_keys = idxs_by_cluster.keys()

    marker_array = MarkerArray()

    if timestamp is None:
        timestamp = rospy.Time.now()

    marker_array.markers.append(
        get_LiDAR_marker(timestamp)
    )

    for id, idx in enumerate(clusters_keys):

        marker = Marker()

        marker.header.frame_id = "/velodyne"
        marker.header.stamp = timestamp

        marker.ns = "cone_markers"

        marker.lifetime = rospy.Duration(VISUALIZATION_LIFETIME)

        marker.action = Marker.ADD
        marker.id = id
        marker.type = marker.CYLINDER

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.4

        marker.color.a = 1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.orientation.w = 1.0

        marker.pose.position.x = \
            nparray_obstacle_xy[
                idxs_by_cluster[idx][-1]
            ][0]

        marker.pose.position.y = \
            nparray_obstacle_xy[
                idxs_by_cluster[idx][-1]
            ][1]

        marker.pose.position.z = 0

        marker_array.markers.append(marker)

    publisher_rviz_markers.publish(marker_array)


# ============================================================
# 点云预处理
# ============================================================

def remove_field_num(a, i):

    names = list(a.dtype.names)

    new_names = names[:i] + names[i+1:]

    return a[new_names]


def preprocessing(msg):

    timestamp = msg.header.stamp

    starray_obstacle_xyz = \
        rnp.point_cloud2.pointcloud2_to_array(msg)

    starray_obstacle_xy = \
        remove_field_num(
            starray_obstacle_xyz,
            2
        )

    nparray_obstacle_xy = \
        starray_obstacle_xy.view(
            np.float32
        ).reshape(
            starray_obstacle_xy.shape + (-1,)
        )

    return timestamp, \
           nparray_obstacle_xy, \
           starray_obstacle_xyz


def filter_vehicle_returns(obstacle_xy, obstacle_xyz):
    """在聚类前去除无效点和车辆本体回波，保持 XY/XYZ 索引一致。"""
    if len(obstacle_xy) == 0:
        return obstacle_xy, obstacle_xyz

    lidar_x = obstacle_xy[:, 0]
    lidar_y = obstacle_xy[:, 1]
    yaw = np.deg2rad(LIDAR_YAW_OFFSET_DEG)
    lateral = LIDAR_Y_SIGN * lidar_y
    vehicle_x = lidar_x * np.cos(yaw) - lateral * np.sin(yaw) + LIDAR_OFFSET_X
    vehicle_y = lidar_x * np.sin(yaw) + lateral * np.cos(yaw) + LIDAR_OFFSET_Y

    valid = (np.isfinite(lidar_x) & np.isfinite(lidar_y) &
             np.isfinite(obstacle_xyz['z']))
    on_vehicle = ((vehicle_x >= -SELF_FILTER_REAR) &
                  (vehicle_x <= SELF_FILTER_FRONT) &
                  (np.abs(vehicle_y) <= SELF_FILTER_HALF_WIDTH))
    keep = valid & ~on_vehicle
    return obstacle_xy[keep], obstacle_xyz[keep]


# ============================================================
# 自适应DBSCAN
# ============================================================

def adaptive_dbscan(points,
                    k=K_NEIGHBORS,
                    min_pts=MIN_PTS):

    """
    自适应DBSCAN

    公式(8)：
    N_e(p) = {q in P | dist(p,q) <= epsilon}

    公式(9)：
    |N_e(p)| >= MinPts

    公式(10)：
    epsilon_i =
        1/k * sum d(p_i,p_j)

    每个点根据自己的k近邻平均距离
    得到局部自适应邻域半径epsilon_i。
    """

    points = np.asarray(points)

    n_points = len(points)

    # 没有点
    if n_points == 0:
        return np.array([], dtype=int), \
               np.array([]), \
               np.array([], dtype=bool)

    # 只有一个点时无法计算k近邻
    if n_points == 1:

        labels = np.array([-1], dtype=int)
        epsilon = np.array([0.0])
        core_points = np.array([False])

        return labels, epsilon, core_points

    # 防止k超过实际点数量
    actual_k = min(
        k,
        n_points - 1
    )

    # ========================================================
    # 计算点与点之间的欧氏距离矩阵
    # ========================================================

    diff = \
        points[:, np.newaxis, :] - \
        points[np.newaxis, :, :]

    distance_matrix = np.sqrt(
        np.sum(
            diff ** 2,
            axis=2
        )
    )

    # ========================================================
    # 公式(10)：计算每个点的自适应epsilon_i
    # ========================================================

    epsilon = np.zeros(
        n_points,
        dtype=float
    )

    for i in range(n_points):

        # 复制当前点到其他所有点的距离
        distances = \
            distance_matrix[i].copy()

        # 排除自身距离0
        distances[i] = np.inf

        # 取k个最近邻
        nearest_distances = \
            np.sort(
                distances
            )[:actual_k]

        # epsilon_i为k近邻平均欧氏距离
        epsilon[i] = np.mean(
            nearest_distances
        )

    # ========================================================
    # 公式(8)：计算每个点的局部邻域
    # ========================================================

    neighborhoods = []

    for i in range(n_points):

        # 注意：
        # distance_matrix[i,i] = 0
        # 因此这里会包含点自身
        neighbors = np.where(
            distance_matrix[i] <= epsilon[i]
        )[0]

        neighborhoods.append(
            neighbors
        )

    # ========================================================
    # 公式(9)：判断核心点
    # ========================================================

    core_points = np.zeros(
        n_points,
        dtype=bool
    )

    for i in range(n_points):

        if len(neighborhoods[i]) >= min_pts:

            core_points[i] = True

    # ========================================================
    # 开始DBSCAN簇扩展
    # ========================================================

    # -1表示噪声点
    labels = np.full(
        n_points,
        -1,
        dtype=int
    )

    visited = np.zeros(
        n_points,
        dtype=bool
    )

    cluster_id = 0

    for i in range(n_points):

        # 已经访问过
        if visited[i]:
            continue

        visited[i] = True

        # 非核心点暂时作为噪声
        if not core_points[i]:
            continue

        # 当前点属于新的聚类
        labels[i] = cluster_id

        seeds = list(
            neighborhoods[i]
        )

        # 用于避免重复加入
        seed_set = set(seeds)

        j = 0

        while j < len(seeds):

            current_point = seeds[j]

            if not visited[current_point]:

                visited[current_point] = True

                # 如果该点也是核心点
                # 则继续扩展其邻域
                if core_points[current_point]:

                    for neighbor in \
                            neighborhoods[current_point]:

                        if neighbor not in seed_set:

                            seeds.append(
                                neighbor
                            )

                            seed_set.add(
                                neighbor
                            )

            # 原来被视为噪声的点
            # 如果密度可达，则加入当前簇
            if labels[current_point] == -1:

                labels[current_point] = \
                    cluster_id

            j += 1

        cluster_id += 1

    return labels, epsilon, core_points


# ============================================================
# 点数量得分
# ============================================================

def calculate_point_number_score(N_G):

    """
    s_N =
    clip(
        (N_G-N_MIN)/(N_MAX-N_MIN),
        0,1
    )
    """

    if N_MAX == N_MIN:
        return 0.0

    s_N = (
        N_G - N_MIN
    ) / float(
        N_MAX - N_MIN
    )

    return np.clip(
        s_N,
        0.0,
        1.0
    )


# ============================================================
# 高度得分
# ============================================================

def calculate_height_score(cluster_xyz):

    """
    H_G = Z_max - Z_min
    """

    z_values = np.asarray(
        cluster_xyz['z']
    ).reshape(-1)

    if len(z_values) == 0:
        return 0.0, 0.0

    H_G = \
        np.max(z_values) - \
        np.min(z_values)

    if H_G < H_MIN:

        s_H = 0.0

    elif H_G <= H_MAX:

        if H_MAX == H_MIN:

            s_H = 0.0

        else:

            s_H = (
                H_G - H_MIN
            ) / float(
                H_MAX - H_MIN
            )

    else:

        s_H = 0.0

    return np.clip(
        s_H,
        0.0,
        1.0
    ), H_G


# ============================================================
# 平均半径得分
# ============================================================

def calculate_radius_score(cluster_xy):

    """
    R_G采用XY平面包围盒等效半径：

    R_G =
    0.5 * sqrt(
        (Xmax-Xmin)^2 +
        (Ymax-Ymin)^2
    )
    """

    cluster_xy = np.asarray(
        cluster_xy
    )

    if len(cluster_xy) == 0:
        return 0.0, 0.0

    x_values = cluster_xy[:, 0]
    y_values = cluster_xy[:, 1]

    x_range = \
        np.max(x_values) - \
        np.min(x_values)

    y_range = \
        np.max(y_values) - \
        np.min(y_values)

    R_G = 0.5 * np.sqrt(
        x_range ** 2 +
        y_range ** 2
    )

    if R_G < R_MIN:

        s_R = 0.0

    elif R_G <= R_MAX:

        if R_MAX == R_MIN:

            s_R = 0.0

        else:

            s_R = (
                R_G - R_MIN
            ) / float(
                R_MAX - R_MIN
            )

    else:

        s_R = 0.0

    return np.clip(
        s_R,
        0.0,
        1.0
    ), R_G


# ============================================================
# 距离衰减得分
# ============================================================

def calculate_distance_score(cluster_xy):

    """
    d = sqrt(x_c^2 + y_c^2)

    s_d =
    clip(
        1-d/D_MAX,
        0,1
    )
    """

    cluster_xy = np.asarray(
        cluster_xy
    )

    if len(cluster_xy) == 0:
        return 0.0, 0.0

    # 聚类中心
    center_x = np.mean(
        cluster_xy[:, 0]
    )

    center_y = np.mean(
        cluster_xy[:, 1]
    )

    # 聚类中心距离雷达的距离
    d = np.sqrt(
        center_x ** 2 +
        center_y ** 2
    )

    if D_MAX <= 0:

        s_d = 0.0

    else:

        s_d = np.clip(
            1.0 -
            d / float(D_MAX),
            0.0,
            1.0
        )

    return s_d, d


# ============================================================
# LiDAR总置信度
# ============================================================

def calculate_lidar_confidence(
        N_G,
        cluster_xy,
        cluster_xyz):

    # 点数量得分
    s_N = \
        calculate_point_number_score(
            N_G
        )

    # 高度得分
    s_H, H_G = \
        calculate_height_score(
            cluster_xyz
        )

    # 平均半径得分
    s_R, R_G = \
        calculate_radius_score(
            cluster_xy
        )

    # 距离衰减得分
    s_d, d = \
        calculate_distance_score(
            cluster_xy
        )

    # ========================================================
    # 雷达总置信度加权融合公式
    #
    # C_lidar =
    # W_N*s_N +
    # W_H*s_H +
    # W_R*s_R +
    # W_D*s_d
    # ========================================================

    C_lidar = (
        W_N * s_N +
        W_H * s_H +
        W_R * s_R +
        W_D * s_d
    )

    C_lidar = np.clip(
        C_lidar,
        0.0,
        1.0
    )

    return {
        's_N': s_N,
        's_H': s_H,
        's_R': s_R,
        's_d': s_d,

        'N_G': N_G,
        'H_G': H_G,
        'R_G': R_G,
        'd': d,

        'C_lidar': C_lidar
    }


# ============================================================
# 发布PoseArray
# ============================================================

def publish_to_perception(pose_array):

    publisher_perception.publish(
        pose_array
    )


# ============================================================
# 主聚类函数
# ============================================================

def compute_clustering(msg):
    """Compute one frame without publishing; retain detection and source stamp."""
    stages = {}
    # NumPy cannot infer reshape((0, -1)) in the legacy preprocessing helper.
    # A valid empty cloud is an observation with no detections, not an exception.
    if msg.width == 0 or msg.height == 0:
        empty = PoseArray()
        empty.header.stamp = msg.header.stamp
        empty.header.frame_id = '/velodyne'
        return dict(pose_array=empty, stage_s={}, input_points=0, filtered_points=0,
                    idxs_by_cluster={}, obstacle_xy=np.empty((0, 2)),
                    obstacle_xyz=np.empty(0), confidences={})
    started = default_timer()
    timestamp, obstacle_xy, obstacle_xyz = preprocessing(msg)
    stages['preprocess'] = max(0.0, default_timer()-started)
    input_points = len(obstacle_xy)

    started = default_timer()
    obstacle_xy, obstacle_xyz = filter_vehicle_returns(obstacle_xy, obstacle_xyz)
    stages['filter'] = max(0.0, default_timer()-started)
    filtered_points = len(obstacle_xy)
    idxs_by_cluster = {}

    started = default_timer()
    if filtered_points:
        labels, epsilon_values, core_samples_mask = adaptive_dbscan(
            obstacle_xy, k=K_NEIGHBORS, min_pts=MIN_PTS)
        n_clusters = len(set(labels))-(1 if -1 in labels else 0)
        idxs_by_cluster = dict((i, []) for i in range(n_clusters))
        for index, label in enumerate(labels):
            if label != -1:
                idxs_by_cluster[label].append(index)
    stages['cluster'] = max(0.0, default_timer()-started)

    # Keep the existing confidence calculation without per-cluster console I/O.
    started = default_timer()
    lidar_confidences = {}
    for cluster_id, indices in idxs_by_cluster.items():
        lidar_confidences[cluster_id] = calculate_lidar_confidence(
            len(indices), obstacle_xy[indices], obstacle_xyz[indices])
    stages['confidence'] = max(0.0, default_timer()-started)

    started = default_timer()
    pose_array = get_PoseArray(idxs_by_cluster, obstacle_xy, obstacle_xyz, timestamp)
    stages['build_output'] = max(0.0, default_timer()-started)
    return dict(pose_array=pose_array, stage_s=stages,
                input_points=input_points, filtered_points=filtered_points,
                idxs_by_cluster=idxs_by_cluster, obstacle_xy=obstacle_xy,
                obstacle_xyz=obstacle_xyz, confidences=lidar_confidences)


class ClusteringNode(object):
    """Keep reception fast and bound pending work to the latest complete frame."""

    def __init__(self):
        global publisher_perception, publisher_rviz_markers, VISUALIZATION_LIFETIME
        self.visualization_enabled = bool(rospy.get_param('~publish_visualization', True))
        self.visualization_hz = float(rospy.get_param('~visualization_hz', 2.0))
        if (math.isnan(self.visualization_hz) or math.isinf(self.visualization_hz) or
                self.visualization_hz <= 0):
            raise ValueError('visualization_hz must be finite and positive')
        VISUALIZATION_LIFETIME = max(0.1, 1.5/self.visualization_hz)
        self.visualization_result = None
        self.last_profile = {}
        self.last_publish_s = None
        self.last_marker_s = None
        self.receive_buff_size = int(rospy.get_param('~receive_buff_size', 1048576))
        if self.receive_buff_size < 65536:
            raise ValueError('receive_buff_size must be at least 65536 bytes')

        # Advertise before subscribing: an immediate incoming cloud must never
        # observe uninitialized publishers, including on a fast replay.
        self.perception_pub = rospy.Publisher('/clustered_points', PoseArray, queue_size=1)
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher('/perception/clustering_status', String, queue_size=1)
        publisher_perception = self.perception_pub
        publisher_rviz_markers = self.marker_pub
        self.worker = LatestFrameWorker(
            source_stamp=lambda msg: msg.header.stamp.to_sec(),
            now_ros=lambda: rospy.Time.now().to_sec(),
            processor=self.process, deliver=self.deliver,
            on_error=self.on_error,
            max_age=float(rospy.get_param('~max_frame_age', 0.4)),
            future_tolerance=0.1)
        self.subscriber = rospy.Subscriber(
            '/ground_segmentation/obstacle_cloud', PointCloud2, self.worker.submit,
            queue_size=1, buff_size=self.receive_buff_size, tcp_nodelay=True)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status, reset=True)
        self.visualization_timer = None
        if self.visualization_enabled:
            self.visualization_timer = rospy.Timer(
                rospy.Duration(1.0/self.visualization_hz), self.publish_visualization,
                reset=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('Clustering latest-frame worker; max_frame_age=%.3fs, buffer=%d bytes',
                      self.worker.max_age, self.receive_buff_size)

    def process(self, msg):
        result = compute_clustering(msg)
        # One atomic assignment; the status timer only copies this small dict.
        self.last_profile = dict(stage_s=result['stage_s'],
                                 input_points=int(result['input_points']),
                                 filtered_points=int(result['filtered_points']),
                                 clusters=len(result['idxs_by_cluster']))
        return result

    def deliver(self, result, msg):
        # The worker checks age again immediately before this call. Preserve the
        # source timestamp, and enqueue perception before optional visualization.
        started = default_timer()
        self.perception_pub.publish(result['pose_array'])
        self.last_publish_s = max(0.0, default_timer()-started)
        # A single optional visualization slot, consumed by a separate timer.
        # Visualization never holds the worker's delivery/stop lock.
        if self.visualization_enabled:
            self.visualization_result = result

    def publish_visualization(self, event):
        result = self.visualization_result
        if result is None or not self.marker_pub.get_num_connections():
            return
        age = rospy.Time.now().to_sec()-result['pose_array'].header.stamp.to_sec()
        if not -0.1 <= age <= self.worker.max_age:
            return
        started = default_timer()
        try:
            publish_markers_to_RVIZ(result['idxs_by_cluster'], result['obstacle_xy'],
                                    result['obstacle_xyz'],
                                    timestamp=result['pose_array'].header.stamp)
        except Exception as error:
            rospy.logwarn_throttle(2.0, 'Clustering visualization failed: %s', error)
        self.last_marker_s = max(0.0, default_timer()-started)

    def on_error(self, error):
        rospy.logwarn_throttle(2.0, 'Clustering frame failed: %s', error)

    def publish_status(self, event):
        status = self.worker.snapshot()
        status['profile'] = dict(self.last_profile)
        status['publish_call_s'] = self.last_publish_s
        status['marker_s'] = self.last_marker_s
        status['max_frame_age_s'] = self.worker.max_age
        status['receive_buff_size'] = self.receive_buff_size
        self.status_pub.publish(String(json.dumps(status, allow_nan=False, sort_keys=True)))
        rospy.loginfo_throttle(2.0, 'Clustering realtime: %s',
                               json.dumps(status, allow_nan=False, sort_keys=True))

    def shutdown(self):
        self.subscriber.unregister()
        self.status_timer.shutdown()
        if self.visualization_timer is not None:
            self.visualization_timer.shutdown()
        self.worker.stop(join_timeout=2.0)


if __name__ == '__main__':

    rospy.init_node(
        'clustering'
    )

    LIDAR_OFFSET_X = rospy.get_param('~lidar_offset_x', LIDAR_OFFSET_X)
    LIDAR_OFFSET_Y = rospy.get_param('~lidar_offset_y', LIDAR_OFFSET_Y)
    LIDAR_Y_SIGN = rospy.get_param('~lidar_y_sign', LIDAR_Y_SIGN)
    LIDAR_YAW_OFFSET_DEG = rospy.get_param('~lidar_yaw_offset_deg', LIDAR_YAW_OFFSET_DEG)
    SELF_FILTER_REAR = rospy.get_param('~self_filter_rear', SELF_FILTER_REAR)
    SELF_FILTER_FRONT = rospy.get_param('~self_filter_front', SELF_FILTER_FRONT)
    SELF_FILTER_HALF_WIDTH = rospy.get_param('~self_filter_half_width', SELF_FILTER_HALF_WIDTH)

    node = ClusteringNode()
    rospy.spin()
