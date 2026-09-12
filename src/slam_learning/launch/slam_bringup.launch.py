import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('slam_learning'),
        'rviz',
        'slam.rviz',
    )
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='slam_learning',
            executable='scan_node',
            name='icp_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'map_resolution': 0.05,
                'map_width': 600,
                'map_height': 600,
                'map_origin_x': -15.0,
                'map_origin_y': -15.0,
                'submap_scan_count': 20,
                'keyframe_translation': 0.05,
                'keyframe_rotation_deg': 2.0,
                'max_icp_error': 0.08,
                'min_icp_matches': 30,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='slam_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            additional_env={
                'LD_PRELOAD': '',
                'GTK_PATH': '',
                'GTK_EXE_PREFIX': '',
                'GIO_EXTRA_MODULES': '',
                'QT_PLUGIN_PATH': '',
                'QML2_IMPORT_PATH': '',
                'LIBGL_DRIVERS_PATH': '',
                'GBM_BACKENDS_PATH': '',
                '__EGL_VENDOR_LIBRARY_DIRS': '',
                'VK_ICD_FILENAMES': '',
                'SNAP_LIBRARY_PATH': '',
                'LIBGL_ALWAYS_SOFTWARE': '0',
                'MESA_GL_VERSION_OVERRIDE': '',
                'LD_LIBRARY_PATH': (
                    '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:'
                    '/opt/ros/humble/opt/rviz_ogre_vendor/lib:'
                    '/opt/ros/humble/lib/x86_64-linux-gnu:'
                    '/opt/ros/humble/lib'
                ),
            },
        ),
    ])
