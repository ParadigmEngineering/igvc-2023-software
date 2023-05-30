import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('para_control')
    print(pkg_share)
    keyboard_control_node_path = os.path.join(pkg_share, 'para_control', 'keyboard_control_node.py')

    return LaunchDescription([
        Node(
            package='para_control',
            executable=FindExecutable(name='python3'),
            arguments=[keyboard_control_node_path],
            name='keyboard_control_node',
            output='screen',
            parameters=[
            ],
        ),
    ])