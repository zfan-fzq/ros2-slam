import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

print("Python:", sys.executable)
print("rclpy:", rclpy.__file__)
print("Node:", Node)
print("String:", String)
print("ROS2 Python environment OK!")