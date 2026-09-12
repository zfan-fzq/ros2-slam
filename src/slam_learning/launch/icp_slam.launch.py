from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_learning',
            executable='scan_node',
            name='icp_laser_odometry',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
