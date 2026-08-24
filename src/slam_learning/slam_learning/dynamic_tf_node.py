import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path

from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster
)


class DynamicTFNode(Node):

    def __init__(self):
        super().__init__('dynamic_tf_node')

        # ==========================================
        # 1. TF发布器
        # ==========================================

        self.tf_broadcaster = TransformBroadcaster(self)

        self.static_tf_broadcaster = (
            StaticTransformBroadcaster(self)
        )

        # ==========================================
        # 2. 创建三条轨迹的Publisher
        # ==========================================

        self.odom_path_publisher = self.create_publisher(
            Path,
            '/odom_path',
            10
        )

        self.slam_path_publisher = self.create_publisher(
            Path,
            '/slam_path',
            10
        )

        self.ground_truth_path_publisher = self.create_publisher(
            Path,
            '/ground_truth_path',
            10
        )

        # ==========================================
        # 3. 创建三条Path消息
        # ==========================================

        self.odom_path = Path()
        self.odom_path.header.frame_id = 'map'

        self.slam_path = Path()
        self.slam_path.header.frame_id = 'map'

        self.ground_truth_path = Path()
        self.ground_truth_path.header.frame_id = 'map'

        # 保存每一个历史odom位姿
        self.odom_history = []

        # ==========================================
        # 4. 理想真实位姿
        # ==========================================

        self.true_x = 0.0
        self.true_y = 0.0
        self.true_yaw = 0.0

        # ==========================================
        # 5. 带漂移的里程计位姿
        # ==========================================

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # ==========================================
        # 6. map → odom修正量
        # ==========================================

        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_yaw = 0.0

        # ==========================================
        # 7. 理想运动参数
        # ==========================================

        self.true_v = 0.5
        self.true_omega = 0.2

        # ==========================================
        # 8. 里程计运动参数
        # 故意加入比例误差
        # ==========================================

        self.odom_v = self.true_v * 1.03
        self.odom_omega = self.true_omega * 0.97

        self.dt = 0.1

        # ==========================================
        # 9. 每隔5秒进行一次SLAM修正
        # ==========================================

        self.correction_interval = 5.0

        self.correction_steps = int(
            self.correction_interval / self.dt
        )

        self.step_count = 0

        # ==========================================
        # 10. 发布静态雷达TF
        # ==========================================

        self.publish_base_to_laser()

        # ==========================================
        # 11. 创建定时器
        # ==========================================

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        self.get_logger().info(
            'SLAM drift demo started'
        )

        self.get_logger().info(
            'Blue: ground truth, '
            'Red: odometry, '
            'Green: SLAM corrected'
        )

    # ==============================================
    # 发布 base_link → laser
    # ==============================================

    def publish_base_to_laser(self):

        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )

        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser'

        transform.transform.translation.x = 0.20
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.15

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(
            transform
        )

    # ==============================================
    # 角度归一化到[-pi, pi]
    # ==============================================

    def normalize_angle(self, angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    # ==============================================
    # 根据yaw计算四元数
    # ==============================================

    def yaw_to_quaternion(self, yaw):

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        return qz, qw

    # ==============================================
    # 创建一个PoseStamped
    # ==============================================

    def create_pose(self, x, y, yaw, now):

        pose = PoseStamped()

        pose.header.stamp = now
        pose.header.frame_id = 'map'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(yaw)

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    # ==============================================
    # 更新机器人位姿
    # ==============================================

    def update_pose(self, x, y, yaw, v, omega):

        x += (
            v
            * self.dt
            * math.cos(yaw)
        )

        y += (
            v
            * self.dt
            * math.sin(yaw)
        )

        yaw += (
            omega
            * self.dt
        )

        yaw = self.normalize_angle(yaw)

        return x, y, yaw

    # ==============================================
    # 模拟SLAM计算map → odom
    # ==============================================

    def calculate_map_to_odom(self):
        """
        已知：

        T_map_base：模拟的真实/SLAM位姿
        T_odom_base：带漂移的里程计位姿

        根据：

        T_map_base =
            T_map_odom × T_odom_base

        得到：

        T_map_odom =
            T_map_base × inverse(T_odom_base)
        """

        # map与odom之间的角度修正
        self.map_to_odom_yaw = self.normalize_angle(
            self.true_yaw - self.odom_yaw
        )

        c = math.cos(self.map_to_odom_yaw)
        s = math.sin(self.map_to_odom_yaw)

        # 将odom中的机器人位置旋转到map方向
        rotated_odom_x = (
            c * self.odom_x
            - s * self.odom_y
        )

        rotated_odom_y = (
            s * self.odom_x
            + c * self.odom_y
        )

        # 计算map → odom的平移修正
        self.map_to_odom_x = (
            self.true_x - rotated_odom_x
        )

        self.map_to_odom_y = (
            self.true_y - rotated_odom_y
        )

        self.get_logger().info(
            'SLAM correction: '
            f'map->odom = '
            f'[{self.map_to_odom_x:.3f}, '
            f'{self.map_to_odom_y:.3f}, '
            f'{math.degrees(self.map_to_odom_yaw):.2f} deg]'
        )

    # ==============================================
    # odom位姿经过map → odom变换
    # ==============================================

    def transform_odom_pose_to_map(
    self,
    odom_x,
    odom_y,
    odom_yaw):
        

        c = math.cos(self.map_to_odom_yaw)
        s = math.sin(self.map_to_odom_yaw)

        map_x = (
            self.map_to_odom_x
            + c * odom_x
            - s * odom_y
        )

        map_y = (
            self.map_to_odom_y
            + s * odom_x
            + c * odom_y
        )

        map_yaw = self.normalize_angle(
            self.map_to_odom_yaw
            + odom_yaw
        )

        return map_x, map_y, map_yaw


    def rebuild_slam_path(self):
        """
        使用最新的map → odom，
        重新计算全部历史SLAM轨迹。
        """

        new_slam_path = Path()
        new_slam_path.header.frame_id = 'map'

        for (
            odom_x,
            odom_y,
            odom_yaw,
            pose_time
        ) in self.odom_history:

            map_x, map_y, map_yaw = (
                self.transform_odom_pose_to_map(
                    odom_x,
                    odom_y,
                    odom_yaw
                )
            )

            corrected_pose = self.create_pose(
                map_x,
                map_y,
                map_yaw,
                pose_time
            )

            new_slam_path.poses.append(
                corrected_pose
            )

        self.slam_path = new_slam_path

    def publish_map_to_odom(self, now):

        transform = TransformStamped()

        transform.header.stamp = now
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'

        transform.transform.translation.x = (
            self.map_to_odom_x
        )

        transform.transform.translation.y = (
            self.map_to_odom_y
        )

        transform.transform.translation.z = 0.0

        qz, qw = self.yaw_to_quaternion(
            self.map_to_odom_yaw
        )

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        return transform

    # ==============================================
    # 发布odom → base_link
    # ==============================================

    def publish_odom_to_base(self, now):

        transform = TransformStamped()

        transform.header.stamp = now
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.odom_x
        transform.transform.translation.y = self.odom_y
        transform.transform.translation.z = 0.0

        qz, qw = self.yaw_to_quaternion(
            self.odom_yaw
        )

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        return transform

    # ==============================================
    # 定时器主循环
    # ==============================================

    def timer_callback(self):

        self.step_count += 1

        # ------------------------------------------
        # 1. 更新理想真实位姿
        # ------------------------------------------

        (
            self.true_x,
            self.true_y,
            self.true_yaw
        ) = self.update_pose(
            self.true_x,
            self.true_y,
            self.true_yaw,
            self.true_v,
            self.true_omega
        )

        # ------------------------------------------
        # 2. 更新带漂移的里程计位姿
        # ------------------------------------------

        (
            self.odom_x,
            self.odom_y,
            self.odom_yaw
        ) = self.update_pose(
            self.odom_x,
            self.odom_y,
            self.odom_yaw,
            self.odom_v,
            self.odom_omega
        )

        # ------------------------------------------
        # 3. 每5秒执行一次SLAM修正
        # ------------------------------------------

        if self.step_count % self.correction_steps == 0:
            self.calculate_map_to_odom()

        # ------------------------------------------
        # 4. 获取当前时间
        # ------------------------------------------

        now = self.get_clock().now().to_msg()

        # ------------------------------------------
        # 5. 保存当前odom历史位姿
        # ------------------------------------------

        self.odom_history.append(
            (
                self.odom_x,
                self.odom_y,
                self.odom_yaw,
                now
            )
        )

        # ------------------------------------------
        # 6. 重新计算完整SLAM轨迹
        # ------------------------------------------

        self.rebuild_slam_path()

        # ------------------------------------------
        # 5. 发布TF
        # ------------------------------------------

        map_to_odom = self.publish_map_to_odom(now)
        odom_to_base = self.publish_odom_to_base(now)

        self.tf_broadcaster.sendTransform([
            map_to_odom,
            odom_to_base
        ])

        # ------------------------------------------
        # 6. 保存三条轨迹
        # ------------------------------------------

        ground_truth_pose = self.create_pose(
            self.true_x,
            self.true_y,
            self.true_yaw,
            now
        )

        odom_pose = self.create_pose(
            self.odom_x,
            self.odom_y,
            self.odom_yaw,
            now
        )

        self.ground_truth_path.poses.append(
            ground_truth_pose
        )

        self.odom_path.poses.append(
            odom_pose
        )

        # ------------------------------------------
        # 7. 更新Path时间戳
        # ------------------------------------------

        self.ground_truth_path.header.stamp = now
        self.odom_path.header.stamp = now
        self.slam_path.header.stamp = now

        # ------------------------------------------
        # 8. 发布Path
        # ------------------------------------------

        self.ground_truth_path_publisher.publish(
            self.ground_truth_path
        )

        self.odom_path_publisher.publish(
            self.odom_path
        )

        self.slam_path_publisher.publish(
            self.slam_path
        )


def main(args=None):

    rclpy.init(args=args)

    node = DynamicTFNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()