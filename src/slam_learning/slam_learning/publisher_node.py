import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SlamPublisher(Node):

    def __init__(self):
        super().__init__('slam_publisher')

        self.publisher_ = self.create_publisher(
            String,
            '/slam_test',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.count = 0

    def timer_callback(self):
        msg = String()

        msg.data = f'LiDAR frame: {self.count}'

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Publish: {msg.data}'
        )

        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    node = SlamPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()