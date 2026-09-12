import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    assignment_share = get_package_share_directory('assignment3_tb3')
    slam_share = get_package_share_directory('slam_learning')

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            assignment_share, 'launch', 'new_world.launch.py'
        )),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            slam_share, 'launch', 'slam_bringup.launch.py'
        )),
    )
    return LaunchDescription([simulator, slam])
