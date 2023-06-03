from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # main course
    goal_gps_coordinates = '42.400577100,-83.130962509;42.400523432,-83.130969819;42.400507588,-83.130969297;42.400453974,-83.130957951'

    # practice course
    # goal_gps_coordinates = '42.400556255,-83.130645144;42.400510946,-83.130640432;42.400465621,-83.130635756'

    node = Node(
        package='your_package',
        executable='examplenode',
        name='your_node',
        parameters=[
            {'goal_gps_coordinates': goal_gps_coordinates}
        ],
        output='screen'
    )