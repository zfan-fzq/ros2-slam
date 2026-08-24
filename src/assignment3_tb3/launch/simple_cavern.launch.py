import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    os.environ["TURTLEBOT3_MODEL"] = "waffle_pi"
    pkg_share = get_package_share_directory("assignment3_tb3")
    tb3_share = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    world = os.path.join(pkg_share, "worlds", "simple_cavern.world")

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s -v2 ", world], "on_exit_shutdown": "true"}.items(),
    )
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-g -v2 ", world]}.items(),
    )
    robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_share, "launch", "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_share, "launch", "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": "-12.0", "y_pose": "0.0"}.items(),
    )
    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle_pi"),
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        SetEnvironmentVariable("MESA_GL_VERSION_OVERRIDE", "3.3"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg_share, "models")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(tb3_share, "models")),
        gz_server, gz_client, robot_state, spawn,
    ])
