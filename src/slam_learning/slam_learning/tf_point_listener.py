import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException
)

from tf2_geometry_msgs import do_transform_point


class TFPointListener(Node):

    def __init__(self):
        super().__init__('tf_point_listener')

        # ==========================================
        # 1. 创建TF Buffer
        # ==========================================
        #
        # Buffer负责保存一段时间内的TF数据。
        # 之后调用lookup_transform()时，
        # 会从Buffer中查询坐标关系。

        self.tf_buffer = Buffer()

        # ==========================================
        # 2. 创建TF Listener
        # ==========================================
        #
        # TransformListener负责监听：
        # /tf
        # /tf_static
        #
        # 收到的数据会自动存入tf_buffer。

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ==========================================
        # 3. 创建Marker发布器
        # ==========================================

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/tf_point_markers',
            10
        )

        # 每0.2秒查询一次TF
        self.timer = self.create_timer(
            0.2,
            self.timer_callback
        )

        self.counter = 0

        self.get_logger().info(
            'TF point listener started'
        )

    def create_sphere_marker(
        self,
        marker_id,
        frame_id,
        x,
        y,
        z,
        red,
        green,
        blue,
        scale
    ):
        """
        创建球形Marker。
        """

        marker = Marker()

        marker.header.stamp = Time().to_msg()

        marker.header.frame_id = frame_id

        marker.frame_locked = True

        marker.ns = 'tf_points'
        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0

        return marker

    def create_text_marker(
        self,
        marker_id,
        x,
        y,
        z,
        text
    ):
        """
        在转换后的点旁边显示map坐标。
        """

        marker = Marker()

        marker.header.stamp = Time().to_msg()

        marker.header.frame_id = 'map'

        marker.ns = 'tf_point_text'
        marker.id = marker_id

        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.30

        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.18

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = text

        return marker

    def timer_callback(self):

        # ==========================================
        # 1. 创建laser坐标系中的点
        # ==========================================
        #
        # 这个点位于雷达前方2米：
        #
        # p_laser = [2, 0, 0]

        point_laser = PointStamped()

        point_laser.header.frame_id = 'laser'

        # 零时间表示查询最新可用TF
        point_laser.header.stamp = Time().to_msg()

        point_laser.point.x = 2.0
        point_laser.point.y = 0.0
        point_laser.point.z = 0.0

        try:

            # ======================================
            # 2. 查询map ← laser
            # ======================================
            #
            # target_frame = map
            # source_frame = laser
            #
            # 返回的变换可以把laser中的数据
            # 转换到map坐标系。

            transform = self.tf_buffer.lookup_transform(
                'map',
                'laser',
                Time()
            )

            # ======================================
            # 3. 执行点坐标转换
            # ======================================

            point_map = do_transform_point(
                point_laser,
                transform
            )

        except TransformException as error:

            self.get_logger().warning(
                f'Cannot transform laser to map: {error}'
            )

            return

        # ==========================================
        # 4. 创建RViz Marker
        # ==========================================

        marker_array = MarkerArray()

        # 红色大球：
        # 原始点在laser坐标系中的表达
        laser_marker = self.create_sphere_marker(
            marker_id=0,
            frame_id='laser',
            x=point_laser.point.x,
            y=point_laser.point.y,
            z=point_laser.point.z,
            red=1.0,
            green=0.0,
            blue=0.0,
            scale=0.18
        )

        # 绿色小球：
        # 同一个物理点在map坐标系中的表达
        map_marker = self.create_sphere_marker(
            marker_id=1,
            frame_id='map',
            x=point_map.point.x,
            y=point_map.point.y,
            z=point_map.point.z,
            red=0.0,
            green=1.0,
            blue=0.0,
            scale=0.10
        )

        coordinate_text = (
            f'map: '
            f'({point_map.point.x:.2f}, '
            f'{point_map.point.y:.2f})'
        )

        text_marker = self.create_text_marker(
            marker_id=2,
            x=point_map.point.x,
            y=point_map.point.y,
            z=point_map.point.z,
            text=coordinate_text
        )

        marker_array.markers.append(
            laser_marker
        )

        marker_array.markers.append(
            map_marker
        )

        marker_array.markers.append(
            text_marker
        )

        self.marker_publisher.publish(
            marker_array
        )

        # ==========================================
        # 5. 每秒输出一次坐标
        # ==========================================

        self.counter += 1

        if self.counter % 5 == 0:

            self.get_logger().info(
                'laser point: '
                f'({point_laser.point.x:.2f}, '
                f'{point_laser.point.y:.2f})'
                '  ->  '
                'map point: '
                f'({point_map.point.x:.2f}, '
                f'{point_map.point.y:.2f})'
            )


def main(args=None):

    rclpy.init(args=args)

    node = TFPointListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()