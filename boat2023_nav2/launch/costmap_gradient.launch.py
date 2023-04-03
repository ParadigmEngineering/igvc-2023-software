from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('boat2023_nav2')
    default_params_file = os.path.join(pkg_share, 'launch', 'params', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=default_params_file,
            description='Path to the costmap configuration file.'
        ),

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='nav2_costmap_2d',
            output='screen',
            namespace='/costmap',
            parameters=[LaunchConfiguration('params')]
        )
    ])
