from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('boat2023_nav2')
    model_path = os.path.join(pkg_share, 'src', 'description', 'boat2023.urdf')
    costmap_params_file = os.path.join(pkg_share, 'params', 'costmap_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'params', 'ekf_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),

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

        DeclareLaunchArgument(
            'model', 
            default_value=model_path,
            description='Path to robot urdf file'
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
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('gui'))
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='nav2_costmap_2d',
            output='screen',
            namespace='/costmap',
            parameters=[
                costmap_params_file,
                {'global_frame': 'odom'},
                {'robot_base_frame': 'ego_vehicle'}
            ],
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[LaunchConfiguration('ekf_params')],
        ),
    ])
