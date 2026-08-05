#!/usr/bin/env python3
"""Remove LV-DOT dynamic obstacles from the raw lidar cloud.

Pipeline:
    raw lidar cloud (sensor frame) --> [this node] --> filtered raw cloud
                       ^                    ^
    LV-DOT dynamic bboxes (world frame) ----+---- /odom (base_link pose)

LV-DOT's dynamic boxes are expressed in the odometry world frame while the raw
cloud is in the lidar sensor frame, so each point is transformed to world
coordinates (T_world<-base from the nearest-in-time /odom message x the static
base->lidar mount) purely for the inside-footprint test; points inside any
box's x,y footprint (full z column -- z is deliberately ignored) are deleted.

Additionally, the robot's own footprint is removed: the raw cloud contains
self-hits which LV-DOT clusters into a (static) box, so the filtered_bboxes
detection closest to base_link -- if within self_box_max_dist -- is treated as
the robot and its points are deleted too (remove_self_box param).
The published cloud keeps the ORIGINAL sensor frame, stamp and point layout,
so any downstream consumer (e.g. FAST-LIO) sees a normal raw cloud.

Fails open: without fresh boxes or odometry the input passes through untouched.

Run:
    ros2 run onboard_detector dynamic_cloud_filter.py
"""
import numpy as np
import rclpy
from collections import deque
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

# T_base_link<-right_lidar from the URDF (base_link -> base_plate_link z+0.085
# -> right_lidar xyz -0.215 0.215 0.363, rpy 0 0.349 2.35619). Matches the
# detector's body_to_lidar in cfg/detector_param.yaml.
DEFAULT_BODY_TO_LIDAR = [-0.664476, -0.707110, -0.241800, -0.215,
                          0.664482, -0.707104,  0.241802,  0.215,
                         -0.341958,  0.0,       0.939715,  0.448,
                          0.0,       0.0,       0.0,       1.0]


class DynamicCloudFilter(Node):
    def __init__(self):
        super().__init__('dynamic_cloud_filter')
        self.declare_parameter('cloud_in', '/livox/lidar_192_168_1_11')
        self.declare_parameter('cloud_out', '/livox/lidar_192_168_1_11_filtered')
        self.declare_parameter('bbox_topic', '/onboard_detector/dynamic_bboxes')
        self.declare_parameter('filtered_bbox_topic', '/onboard_detector/filtered_bboxes')
        self.declare_parameter('odom_topic', '/odom')
        # remove the filtered_bboxes detection closest to base_link (the robot's
        # own footprint cluster), but only if its center is within this distance
        self.declare_parameter('remove_self_box', True)
        self.declare_parameter('self_box_max_dist', 1.0)
        self.declare_parameter('body_to_lidar', DEFAULT_BODY_TO_LIDAR)
        # inflate each footprint by this much (m); LV-DOT boxes are tight and
        # lag a moving obstacle slightly, so cover the leading edge
        self.declare_parameter('margin', 0.25)
        # ignore boxes older than this (s); protects against a dead detector
        self.declare_parameter('box_timeout', 0.5)
        # max |cloud stamp - odom stamp| to trust the pose (s)
        self.declare_parameter('odom_timeout', 0.3)

        filtered_bbox_topic = self.get_parameter('filtered_bbox_topic').value
        self.remove_self_box = bool(self.get_parameter('remove_self_box').value)
        self.self_box_max_dist = float(self.get_parameter('self_box_max_dist').value)
        cloud_in = self.get_parameter('cloud_in').value
        cloud_out = self.get_parameter('cloud_out').value
        bbox_topic = self.get_parameter('bbox_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.margin = float(self.get_parameter('margin').value)
        self.box_timeout = float(self.get_parameter('box_timeout').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.T_body_lidar = np.array(
            self.get_parameter('body_to_lidar').value, dtype=np.float64).reshape(4, 4)

        self.boxes = np.zeros((0, 4), np.float32)  # rows: cx, cy, half_x, half_y
        self.boxes_stamp = None
        self.fboxes = np.zeros((0, 4), np.float32)  # filtered (all) detections
        self.fboxes_stamp = None
        self.self_removed_total = 0
        self.odom_buf = deque(maxlen=60)  # (stamp_sec, T_world_body)
        self.removed_total = 0

        # best-effort matches both reliable and best-effort cloud publishers
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(MarkerArray, bbox_topic, self.box_cb, 10)
        if self.remove_self_box:
            self.create_subscription(MarkerArray, filtered_bbox_topic, self.fbox_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 25)
        self.create_subscription(PointCloud2, cloud_in, self.cloud_cb, sensor_qos)
        self.pub = self.create_publisher(PointCloud2, cloud_out, 10)

        self.get_logger().info(
            f'filtering raw [{cloud_in}] with boxes from [{bbox_topic}] + pose '
            f'from [{odom_topic}] -> [{cloud_out}] (margin {self.margin} m)')

    @staticmethod
    def _parse_boxes(msg: MarkerArray):
        # LV-DOT publishes one LINE_LIST marker per box: pose.position holds the
        # box center (x, y) and the line points are corner offsets, so the
        # footprint half-extents are the max |x| / |y| over the points.
        rows = []
        for m in msg.markers:
            if not m.points:
                continue
            half_x = max(abs(p.x) for p in m.points)
            half_y = max(abs(p.y) for p in m.points)
            rows.append((m.pose.position.x, m.pose.position.y, half_x, half_y))
        return np.array(rows, np.float32).reshape(-1, 4)

    def box_cb(self, msg: MarkerArray):
        self.boxes = self._parse_boxes(msg)
        self.boxes_stamp = self.get_clock().now()

    def fbox_cb(self, msg: MarkerArray):
        self.fboxes = self._parse_boxes(msg)
        self.fboxes_stamp = self.get_clock().now()

    def fboxes_fresh(self):
        if self.fboxes_stamp is None or len(self.fboxes) == 0:
            return False
        age = (self.get_clock().now() - self.fboxes_stamp).nanoseconds * 1e-9
        return age <= self.box_timeout

    def self_box_for(self, base_xy):
        """The filtered detection closest to base_link, if close enough to be
        the robot's own footprint cluster."""
        if not (self.remove_self_box and self.fboxes_fresh()):
            return None
        d = np.hypot(self.fboxes[:, 0] - base_xy[0], self.fboxes[:, 1] - base_xy[1])
        i = int(np.argmin(d))
        if d[i] > self.self_box_max_dist:
            return None
        return self.fboxes[i]

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        t = msg.pose.pose.position
        x, y, z, w = q.x, q.y, q.z, q.w
        T = np.eye(4)
        T[:3, :3] = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        T[:3, 3] = (t.x, t.y, t.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_buf.append((stamp, T))

    def boxes_fresh(self):
        if self.boxes_stamp is None or len(self.boxes) == 0:
            return False
        age = (self.get_clock().now() - self.boxes_stamp).nanoseconds * 1e-9
        return age <= self.box_timeout

    def lidar_pose_for(self, stamp_sec):
        """(T_world<-lidar, base_link xy) from the odometry nearest to the
        cloud stamp, or (None, None)."""
        if not self.odom_buf:
            return None, None
        stamp, T_world_body = min(self.odom_buf, key=lambda e: abs(e[0] - stamp_sec))
        if abs(stamp - stamp_sec) > self.odom_timeout:
            return None, None
        return T_world_body @ self.T_body_lidar, T_world_body[:2, 3]

    def cloud_cb(self, msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0:
            self.pub.publish(msg)
            return
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        T, base_xy = self.lidar_pose_for(stamp_sec)
        if T is None:
            self.pub.publish(msg)  # fail open: no usable pose
            return

        boxes = [b for b in self.boxes] if self.boxes_fresh() else []
        n_dyn = len(boxes)
        self_box = self.self_box_for(base_xy)
        if self_box is not None:
            boxes.append(self_box)
        if not boxes:
            self.pub.publish(msg)
            return

        offsets = {f.name: f.offset for f in msg.fields}
        pts = np.frombuffer(msg.data, count=n, dtype=np.dtype({
            'names': ['x', 'y', 'z'], 'formats': ['<f4', '<f4', '<f4'],
            'offsets': [offsets['x'], offsets['y'], offsets['z']],
            'itemsize': msg.point_step}))

        # sensor frame -> world frame (only x,y needed for the footprint test)
        R, t = T[:3, :3], T[:3, 3]
        wx = R[0, 0] * pts['x'] + R[0, 1] * pts['y'] + R[0, 2] * pts['z'] + t[0]
        wy = R[1, 0] * pts['x'] + R[1, 1] * pts['y'] + R[1, 2] * pts['z'] + t[1]

        keep = np.ones(n, dtype=bool)
        for cx, cy, half_x, half_y in boxes:
            keep &= ~((np.abs(wx - cx) <= half_x + self.margin) &
                      (np.abs(wy - cy) <= half_y + self.margin))

        removed = int(n - keep.sum())
        if removed == 0:
            self.pub.publish(msg)
            return

        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)[keep]
        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.height = 1
        out.width = rows.shape[0]
        out.row_step = out.width * out.point_step
        out.is_dense = msg.is_dense
        out.data = rows.tobytes()
        self.pub.publish(out)

        self.removed_total += removed
        self.get_logger().info(
            f'removed {removed} pts ({n_dyn} dynamic box(es)'
            f'{", robot footprint" if self_box is not None else ""}) '
            f'({self.removed_total} total)', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
