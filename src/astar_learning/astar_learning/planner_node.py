import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException

from astar_learning.A8_star import astar

from astar_learning.map_utils import (
    occupancy_to_grid,
    inflate_obstacles,
    world_to_grid,
    grid_to_world,
)

class PlannerNode(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # ---------- Planner State ----------
        self.width = None
        self.height = None
        self.resolution = None

        self.origin_x = None
        self.origin_y = None

        self.map_frame = None

        self.raw_grid = None
        self.planning_grid = None

        # ---------- Robot Parameters ----------
        self.robot_radius = 0.20
        self.safety_margin = 0.10

        self.goal_x = None
        self.goal_y = None

        # ---------- ROS2 Interface ----------
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # ---------- TF ----------
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.path_pub = self.create_publisher(
            Path,
            '/astar_path',
            10
        )

        self.planning_map_pub = self.create_publisher(
            OccupancyGrid,
            '/planning_map',
            10
        )


    def map_callback(self, msg):

        # 1. 保存地图 metadata
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution

        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        self.map_frame = msg.header.frame_id

        # 2. ROS2 OccupancyGrid → A* raw grid
        self.raw_grid = occupancy_to_grid(
            msg.data,
            self.width,
            self.height
        )

        # 3. 机器人实际安全半径：米 → cell
        inflation_distance = (
            self.robot_radius + self.safety_margin
        )

        inflation_radius = math.ceil(
            inflation_distance / self.resolution
        )

        # 4. 地图膨胀
        self.planning_grid = inflate_obstacles(
            self.raw_grid,
            inflation_radius
        )

        self.publish_planning_map(msg)

        self.get_logger().info(
            f'Map received: '
            f'{self.width}x{self.height}, '
            f'resolution={self.resolution:.3f}, '
            f'inflation={inflation_radius} cells'
        )

    def publish_planning_map(self, original_msg):
        planning_msg = OccupancyGrid()

        # planning_map 与原始 /map 使用完全相同的坐标信息
        planning_msg.header = original_msg.header
        planning_msg.info = original_msg.info

        data = []

        for row in self.planning_grid:
            for cell in row:
                if cell == 1:
                    data.append(100)   # 障碍物
                else:
                    data.append(0)     # 自由空间

        planning_msg.data = data

        self.planning_map_pub.publish(planning_msg)

    def goal_callback(self, msg):

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.get_logger().info(
            f'Goal received: '
            f'x={self.goal_x:.2f}, '
            f'y={self.goal_y:.2f}, '
            f'frame={msg.header.frame_id}'
        )

        self.plan()


    def get_robot_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                'base_link',
                Time()
            )

        except TransformException as e:

            self.get_logger().warning(
                f'Cannot get robot pose: {e}'
            )

            return None

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        return robot_x, robot_y
    

    def is_valid_cell(self, cell):

        row, col = cell

        if not (
            0 <= row < self.height
            and
            0 <= col < self.width
        ):
            return False

        if (self.planning_grid[row][col] == 1 or
            self.planning_grid[row][col] == -1):
            return False

        return True
    

    def plan(self):

        # 1. 检查地图是否已经准备好
        if self.planning_grid is None:
            self.get_logger().warning(
                'Cannot plan: map is not ready.'
            )
            return

        # 2. 检查目标是否已经收到
        if self.goal_x is None or self.goal_y is None:
            self.get_logger().warning(
                'Cannot plan: goal is not ready.'
            )
            return

        # 3. 从 TF 获取机器人当前世界坐标
        robot_pose = self.get_robot_pose()

        if robot_pose is None:
            return

        robot_x, robot_y = robot_pose

        # 4. Robot world position → A* start grid
        start = world_to_grid(
            robot_x,
            robot_y,
            self.origin_x,
            self.origin_y,
            self.resolution
        )

        # 5. Goal world position → A* goal grid
        goal = world_to_grid(
            self.goal_x,
            self.goal_y,
            self.origin_x,
            self.origin_y,
            self.resolution
        )

        start_row, start_col = start

        self.get_logger().info(
            f'Robot world pose: x={robot_x:.3f}, y={robot_y:.3f}'
        )

        self.get_logger().info(
            f'Map origin: x={self.origin_x:.3f}, y={self.origin_y:.3f}, '
            f'resolution={self.resolution:.3f}'
        )

        self.get_logger().info(
            f'Start cell: {start}, '
            f'raw={self.raw_grid[start_row][start_col]}, '
            f'planning={self.planning_grid[start_row][start_col]}'
        )

        min_dist_sq = None
        nearest_obstacle = None

        for r in range(self.height):
            for c in range(self.width):

                # 只找原始真实障碍物
                if self.raw_grid[r][c] == 1:

                    dr = r - start_row
                    dc = c - start_col
                    dist_sq = dr * dr + dc * dc

                    if min_dist_sq is None or dist_sq < min_dist_sq:
                        min_dist_sq = dist_sq
                        nearest_obstacle = (r, c)

        if nearest_obstacle is not None:
            distance_cells = math.sqrt(min_dist_sq)
            distance_m = distance_cells * self.resolution

            self.get_logger().info(
                f'Nearest obstacle: {nearest_obstacle}, '
                f'distance={distance_cells:.2f} cells '
                f'({distance_m:.3f} m)'
            )

        # 6. 检查 start / goal 是否在地图范围内
        if not self.is_valid_cell(start):
            self.get_logger().warning(
                f'Cannot plan: start {start} is invalid.'
            )
            return

        if not self.is_valid_cell(goal):
            self.get_logger().warning(
                f'Cannot plan: goal {goal} is invalid.'
            )
            return

        # 7. 调用我们自己的 A*
        path = astar(
            self.planning_grid,
            start,
            goal
        )

        # 8. A* 没找到路径
        if path is None:
            self.get_logger().warning(
                f'No path found: start={start}, goal={goal}'
            )
            return

        self.get_logger().info(
            f'Path found: '
            f'start={start}, '
            f'goal={goal}, '
            f'length={len(path)}'
        )

        # 9. 把 A* path 发回 ROS2
        self.publish_path(path)

    def publish_path(self, path):

        # 1. 创建整条路径消息
        path_msg = Path()

        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # 2. A* 的每一个 grid cell → 一个 PoseStamped
        for row, col in path:

            # grid坐标 → world坐标
            x, y = grid_to_world(
                row,
                col,
                self.origin_x,
                self.origin_y,
                self.resolution
            )

            # 创建一个路径点
            pose = PoseStamped()

            pose.header.frame_id = self.map_frame
            pose.header.stamp = path_msg.header.stamp

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # 第一版A*没有路径朝向
            # 因此使用单位四元数，表示0°旋转
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            # 加入整条Path
            path_msg.poses.append(pose)

        # 3. 发布给ROS2
        self.path_pub.publish(path_msg)

        self.get_logger().info(
            f'Published path with {len(path_msg.poses)} poses.'
        )



def main(args=None):

    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()