import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanNode(Node):

    def __init__(self):
        super().__init__('scan_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Scan node started.')

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

        self.get_logger().info(
            f'LaserScan received: '
            f'{len(msg.ranges)} beams -> '
            f'{len(points)} valid points'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ScanNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()