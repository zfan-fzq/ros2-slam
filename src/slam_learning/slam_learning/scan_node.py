import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class ScanNode(Node):

    def __init__(self):
        super().__init__('scan_node')

        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 600)
        self.declare_parameter('map_height', 600)
        self.declare_parameter('map_origin_x', -15.0)
        self.declare_parameter('map_origin_y', -15.0)
        self.declare_parameter('submap_scan_count', 20)
        self.declare_parameter('keyframe_translation', 0.05)
        self.declare_parameter('keyframe_rotation_deg', 2.0)
        self.declare_parameter('max_icp_error', 0.08)
        self.declare_parameter('min_icp_matches', 30)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/slam_pose',
            10
        )
        self.path_publisher = self.create_publisher(
            Path,
            '/slam_path',
            10
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map', map_qos
        )
        self.submap_publisher = self.create_publisher(
            PointCloud2, '/submap_points', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.previous_points = None
        self.latest_odom_pose = None
        self.previous_scan_odom_pose = None
        self.scan_count = 0
        self.accepted_update_count = 0
        self.rejected_update_count = 0

        self.global_rotation = np.eye(2)
        self.global_translation = np.zeros(2)
        self.path = Path()
        self.path.header.frame_id = 'map'

        self.max_icp_error = float(
            self.get_parameter('max_icp_error').value
        )
        self.min_icp_matches = int(
            self.get_parameter('min_icp_matches').value
        )
        self.max_translation_step = 0.30
        self.max_rotation_step = math.radians(20.0)
        self.max_path_length = 5000

        self.map_resolution = float(
            self.get_parameter('map_resolution').value
        )
        self.map_width = int(self.get_parameter('map_width').value)
        self.map_height = int(self.get_parameter('map_height').value)
        self.map_origin_x = float(
            self.get_parameter('map_origin_x').value
        )
        self.map_origin_y = float(
            self.get_parameter('map_origin_y').value
        )
        submap_scan_count = int(
            self.get_parameter('submap_scan_count').value
        )
        self.keyframe_translation = float(
            self.get_parameter('keyframe_translation').value
        )
        self.keyframe_rotation = math.radians(float(
            self.get_parameter('keyframe_rotation_deg').value
        ))
        self.log_odds = np.zeros(
            (self.map_height, self.map_width), dtype=np.float32
        )
        self.observed = np.zeros(
            (self.map_height, self.map_width), dtype=bool
        )
        self.submap_scans = deque(maxlen=max(1, submap_scan_count))
        self.last_keyframe_translation = None
        self.last_keyframe_yaw = None
        self.keyframe_count = 0
        self.occupied_log_odds = 0.85
        self.free_log_odds = -0.40
        self.min_log_odds = -5.0
        self.max_log_odds = 5.0

        self.get_logger().info(
            'ICP laser odometry started: scan=/scan, odom=/odom, '
            'pose=/slam_pose, path=/slam_path, map=/map, '
            'submap=/submap_points'
        )

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            )
        )
        self.latest_odom_pose = np.array([position.x, position.y, yaw])

    @staticmethod
    def rotation_matrix(yaw):
        return np.array([
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)]
        ])

    @classmethod
    def odom_point_cloud_initial_guess(cls, previous_pose, current_pose):
        """Convert odometry motion to a point-cloud alignment guess."""
        if previous_pose is None or current_pose is None:
            return np.eye(2), np.zeros(2)

        previous_rotation = cls.rotation_matrix(previous_pose[2])
        current_rotation = cls.rotation_matrix(current_pose[2])

        robot_rotation = previous_rotation.T @ current_rotation
        robot_translation = previous_rotation.T @ (
            current_pose[:2] - previous_pose[:2]
        )

        point_rotation = robot_rotation.T
        point_translation = -point_rotation @ robot_translation
        return point_rotation, point_translation

    @staticmethod
    def find_correspondences(source_points, target_points):
        """Find the nearest target point for every source point."""
        if len(source_points) == 0 or len(target_points) == 0:
            return (
                np.empty((0, 2)),
                np.empty(0),
                np.empty(0, dtype=int)
            )

        # Shape: (number of source points, number of target points, 2)
        differences = (
            source_points[:, np.newaxis, :]
            - target_points[np.newaxis, :, :]
        )

        squared_distances = np.sum(differences ** 2, axis=2)
        target_indices = np.argmin(squared_distances, axis=1)

        matched_target_points = target_points[target_indices]
        distances = np.sqrt(
            squared_distances[np.arange(len(source_points)), target_indices]
        )

        return matched_target_points, distances, target_indices

    @staticmethod
    def best_fit_transform(source_points, target_points):
        """Return the 2D rigid transform that maps source onto target."""
        if len(source_points) != len(target_points) or len(source_points) < 2:
            raise ValueError('At least two paired points are required.')

        source_centroid = np.mean(source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)

        source_centered = source_points - source_centroid
        target_centered = target_points - target_centroid

        covariance = source_centered.T @ target_centered
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T

        # A rigid transform may rotate but must not mirror the point cloud.
        if np.linalg.det(rotation) < 0:
            vt[-1, :] *= -1
            rotation = vt.T @ u.T

        translation = target_centroid - rotation @ source_centroid
        return rotation, translation

    @classmethod
    def icp(
        cls,
        source_points,
        target_points,
        max_iterations=30,
        tolerance=1e-4,
        max_correspondence_distance=0.35,
        initial_rotation=None,
        initial_translation=None
    ):
        """
        Align source to target with point-to-point ICP.

        Returns (rotation, translation, error, iterations, matches, converged).
        The returned transform satisfies approximately:
            target = rotation @ source + translation
        """
        if len(source_points) < 3 or len(target_points) < 3:
            return None

        if initial_rotation is None:
            initial_rotation = np.eye(2)
        if initial_translation is None:
            initial_translation = np.zeros(2)

        transformed_source = (
            initial_rotation @ source_points.T
        ).T + initial_translation
        total_rotation = initial_rotation.copy()
        total_translation = initial_translation.copy()
        previous_error = float('inf')
        converged = False
        iterations = 0

        for iteration in range(1, max_iterations + 1):
            matched_targets, distances, _ = cls.find_correspondences(
                transformed_source,
                target_points
            )

            valid = distances < max_correspondence_distance
            match_count = int(np.count_nonzero(valid))

            if match_count < 3:
                return None

            incremental_rotation, incremental_translation = (
                cls.best_fit_transform(
                    transformed_source[valid],
                    matched_targets[valid]
                )
            )

            transformed_source = (
                incremental_rotation @ transformed_source.T
            ).T + incremental_translation

            total_translation = (
                incremental_rotation @ total_translation
                + incremental_translation
            )
            total_rotation = incremental_rotation @ total_rotation

            mean_error = float(np.mean(distances[valid]))
            iterations = iteration

            if abs(previous_error - mean_error) < tolerance:
                converged = True
                break

            previous_error = mean_error

        _, final_distances, _ = cls.find_correspondences(
            transformed_source,
            target_points
        )
        final_valid = final_distances < max_correspondence_distance
        final_match_count = int(np.count_nonzero(final_valid))

        if final_match_count < 3:
            return None

        final_error = float(np.mean(final_distances[final_valid]))

        return (
            total_rotation,
            total_translation,
            final_error,
            iterations,
            final_match_count,
            converged
        )

    @staticmethod
    def invert_point_cloud_transform(rotation, translation):
        """Convert previous-points -> current-points into robot motion."""
        robot_rotation = rotation.T
        robot_translation = -robot_rotation @ translation
        return robot_rotation, robot_translation

    def accept_icp_update(
        self,
        error,
        match_count,
        converged,
        robot_translation,
        robot_yaw
    ):
        return (
            converged
            and error <= self.max_icp_error
            and match_count >= self.min_icp_matches
            and np.linalg.norm(robot_translation)
            <= self.max_translation_step
            and abs(robot_yaw) <= self.max_rotation_step
        )

    def integrate_robot_motion(self, rotation, translation):
        self.global_translation = (
            self.global_translation
            + self.global_rotation @ translation
        )
        self.global_rotation = self.global_rotation @ rotation

    def publish_pose_and_path(self, stamp):
        yaw = math.atan2(
            self.global_rotation[1, 0],
            self.global_rotation[0, 0]
        )

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(self.global_translation[0])
        pose.pose.position.y = float(self.global_translation[1])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.pose_publisher.publish(pose)

        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length:]
        self.path_publisher.publish(self.path)
        self.publish_map_to_odom_transform(stamp)

    def publish_map_to_odom_transform(self, stamp):
        """Broadcast map-to-odom using the SLAM and odometry poses."""
        if self.latest_odom_pose is None:
            map_to_odom_rotation = self.global_rotation
            map_to_odom_translation = self.global_translation
        else:
            odom_rotation = self.rotation_matrix(self.latest_odom_pose[2])
            map_to_odom_rotation = self.global_rotation @ odom_rotation.T
            map_to_odom_translation = (
                self.global_translation
                - map_to_odom_rotation @ self.latest_odom_pose[:2]
            )

        yaw = math.atan2(
            map_to_odom_rotation[1, 0], map_to_odom_rotation[0, 0]
        )
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = float(
            map_to_odom_translation[0]
        )
        transform.transform.translation.y = float(
            map_to_odom_translation[1]
        )
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def world_to_grid(self, x, y):
        column = int(math.floor(
            (x - self.map_origin_x) / self.map_resolution
        ))
        row = int(math.floor(
            (y - self.map_origin_y) / self.map_resolution
        ))
        if 0 <= column < self.map_width and 0 <= row < self.map_height:
            return column, row
        return None

    @staticmethod
    def bresenham(x0, y0, x1, y1):
        """Return all integer grid cells on a ray, including both ends."""
        cells = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx + dy
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            doubled_error = 2 * error
            if doubled_error >= dy:
                error += dy
                x0 += step_x
            if doubled_error <= dx:
                error += dx
                y0 += step_y
        return cells

    def transform_scan_to_map(self, points):
        return (
            self.global_rotation @ points.T
        ).T + self.global_translation

    def should_add_keyframe(self):
        if self.last_keyframe_translation is None:
            return True
        translation_delta = np.linalg.norm(
            self.global_translation - self.last_keyframe_translation
        )
        yaw = math.atan2(
            self.global_rotation[1, 0], self.global_rotation[0, 0]
        )
        yaw_delta = math.atan2(
            math.sin(yaw - self.last_keyframe_yaw),
            math.cos(yaw - self.last_keyframe_yaw),
        )
        return (
            translation_delta >= self.keyframe_translation
            or abs(yaw_delta) >= self.keyframe_rotation
        )

    def update_occupancy_grid(self, map_points):
        robot_cell = self.world_to_grid(
            self.global_translation[0], self.global_translation[1]
        )
        if robot_cell is None:
            self.get_logger().warning(
                'Robot is outside the fixed map; skipping map update.'
            )
            return

        robot_column, robot_row = robot_cell
        for point in map_points:
            endpoint = self.world_to_grid(point[0], point[1])
            if endpoint is None:
                continue
            endpoint_column, endpoint_row = endpoint
            ray = self.bresenham(
                robot_column,
                robot_row,
                endpoint_column,
                endpoint_row,
            )
            for column, row in ray[:-1]:
                self.log_odds[row, column] += self.free_log_odds
                self.observed[row, column] = True
            self.log_odds[endpoint_row, endpoint_column] += (
                self.occupied_log_odds
            )
            self.observed[endpoint_row, endpoint_column] = True

        np.clip(
            self.log_odds,
            self.min_log_odds,
            self.max_log_odds,
            out=self.log_odds,
        )

    def publish_map(self, stamp):
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = 'map'
        grid.info.resolution = self.map_resolution
        grid.info.width = self.map_width
        grid.info.height = self.map_height
        grid.info.origin.position.x = self.map_origin_x
        grid.info.origin.position.y = self.map_origin_y
        grid.info.origin.orientation.w = 1.0

        probability = 1.0 / (1.0 + np.exp(-self.log_odds))
        occupancy = np.full(self.log_odds.shape, -1, dtype=np.int8)
        occupancy[self.observed] = np.rint(
            probability[self.observed] * 100.0
        ).astype(np.int8)
        grid.data = occupancy.ravel().tolist()
        self.map_publisher.publish(grid)

    def publish_submap(self, stamp):
        if not self.submap_scans:
            return
        points = np.concatenate(tuple(self.submap_scans), axis=0)
        xyz_points = np.column_stack((points, np.zeros(len(points))))
        header = Header(stamp=stamp, frame_id='map')
        cloud = point_cloud2.create_cloud_xyz32(header, xyz_points)
        self.submap_publisher.publish(cloud)

    def add_keyframe(self, points, stamp):
        map_points = self.transform_scan_to_map(points)
        self.submap_scans.append(map_points)
        self.update_occupancy_grid(map_points)
        self.publish_map(stamp)
        self.publish_submap(stamp)
        self.last_keyframe_translation = self.global_translation.copy()
        self.last_keyframe_yaw = math.atan2(
            self.global_rotation[1, 0], self.global_rotation[0, 0]
        )
        self.keyframe_count += 1

    def scan_callback(self, msg):

        points = []

        for i, r in enumerate(msg.ranges):

            if not math.isfinite(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            points.append([x, y])

        points = np.array(points)

        self.scan_count += 1

        if self.previous_points is None:
            self.previous_points = points.copy()
            if self.latest_odom_pose is not None:
                self.previous_scan_odom_pose = self.latest_odom_pose.copy()

            self.publish_pose_and_path(msg.header.stamp)
            self.add_keyframe(points, msg.header.stamp)

            self.get_logger().info(
                f'First scan stored: {len(points)} points'
            )
            return

        current_points = points

        initial_rotation, initial_translation = (
            self.odom_point_cloud_initial_guess(
                self.previous_scan_odom_pose,
                self.latest_odom_pose
            )
        )

        result = self.icp(
            self.previous_points,
            current_points,
            initial_rotation=initial_rotation,
            initial_translation=initial_translation
        )

        if result is None:
            self.get_logger().warning(
                f'ICP failed for scan pair {self.scan_count}: '
                'not enough valid correspondences.'
            )
            self.previous_points = current_points.copy()
            if self.latest_odom_pose is not None:
                self.previous_scan_odom_pose = self.latest_odom_pose.copy()
            return

        (
            rotation,
            translation,
            mean_error,
            iterations,
            match_count,
            converged
        ) = result

        robot_rotation, robot_translation = (
            self.invert_point_cloud_transform(rotation, translation)
        )
        robot_yaw = math.atan2(
            robot_rotation[1, 0],
            robot_rotation[0, 0]
        )

        accepted = self.accept_icp_update(
            mean_error,
            match_count,
            converged,
            robot_translation,
            robot_yaw
        )

        if accepted:
            self.integrate_robot_motion(
                robot_rotation,
                robot_translation
            )
            self.accepted_update_count += 1
            self.publish_pose_and_path(msg.header.stamp)
            if self.should_add_keyframe():
                self.add_keyframe(current_points, msg.header.stamp)
        else:
            self.rejected_update_count += 1

        global_yaw = math.atan2(
            self.global_rotation[1, 0],
            self.global_rotation[0, 0]
        )

        self.get_logger().info(
            f'ICP pair {self.scan_count}: '
            f'robot_dx={robot_translation[0]:+.4f} m, '
            f'robot_dy={robot_translation[1]:+.4f} m, '
            f'robot_dyaw={math.degrees(robot_yaw):+.3f} deg, '
            f'error={mean_error:.4f} m, '
            f'matches={match_count}, '
            f'iterations={iterations}, '
            f'converged={converged}, '
            f'accepted={accepted}, '
            f'pose=({self.global_translation[0]:+.3f}, '
            f'{self.global_translation[1]:+.3f}, '
            f'{math.degrees(global_yaw):+.2f} deg)'
        )

        self.previous_points = current_points.copy()
        if self.latest_odom_pose is not None:
            self.previous_scan_odom_pose = self.latest_odom_pose.copy()


def main(args=None):

    rclpy.init(args=args)

    node = ScanNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
