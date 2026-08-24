import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SlamSubscriber(Node):

    def __init__(self):
        super().__init__('slam_subscriber')

        self.subscription = self.create_subscription(
            String,
            '/slam_test',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Receive: {msg.data}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SlamSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()