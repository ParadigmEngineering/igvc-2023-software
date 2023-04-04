from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('boat2023_nav2')
    costmap_params_file = os.path.join(pkg_share, 'params', 'costmap_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'params', 'ekf_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'costmap_params',
            default_value=costmap_params_file,
            description='Path to the costmap configuration file.'
        ),

        DeclareLaunchArgument(
            'ekf_params',
            default_value=ekf_params_file,
            description='Path to the EKF parameters file.'
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "ego_vehicle"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "ego_vehicle", "ego_vehicle/imu"],
            output="screen",
        ),

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='nav2_costmap_2d',
            output='screen',
            namespace='/costmap',
            parameters=[LaunchConfiguration('costmap_params')],
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[LaunchConfiguration('ekf_params')],
        ),
    ])
