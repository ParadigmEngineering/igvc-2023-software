import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('para_ai')
    resize_node_path = os.path.join(pkg_share, 'para_ai', 'resize_node.py')
    bev_node_path = os.path.join(pkg_share, 'para_ai', 'bev_inference_node.py')
    calibration_file_path = os.path.join(pkg_share, 'config', 'camera_calibration.yaml')

    return LaunchDescription([
        Node(
            package='para_ai',
            executable=FindExecutable(name='python3'),
            arguments=[resize_node_path],
            name='resize_node',
            output='screen',
            parameters=[
                {'input_topic': 'carla/ego_vehicle/cam_f/image'},
                {'output_topic': 'para_ai/cam_f/image'},
            ],
        ),

        Node(
            package='para_ai',
            executable=FindExecutable(name='python3'),
            arguments=[resize_node_path],
            name='resize_node',
            output='screen',
            parameters=[
                {'input_topic': 'carla/ego_vehicle/cam_fr/image'},
                {'output_topic': 'para_ai/cam_fr/image'},
            ],
        ),

        Node(
            package='para_ai',
            executable=FindExecutable(name='python3'),
            arguments=[resize_node_path],
            name='resize_node',
            output='screen',
            parameters=[
                {'input_topic': 'carla/ego_vehicle/cam_fl/image'},
                {'output_topic': 'para_ai/cam_fl/image'},
            ],
        ),

        Node(
            package='para_ai',
            executable=FindExecutable(name='python3'),
            arguments=[resize_node_path],
            name='resize_node',
            output='screen',
            parameters=[
                {'input_topic': 'carla/ego_vehicle/cam_b/image'},
                {'output_topic': 'para_ai/cam_b/image'},
            ],
        ),

        Node(
            package='para_ai',
            executable=FindExecutable(name='python3'),
            arguments=[resize_node_path],
            name='resize_node',
            output='screen',
            parameters=[
                {'input_topic': 'carla/ego_vehicle/cam_bl/image'},
                {'output_topic': 'para_ai/cam_bl/image'},
            ],
        ),

        Node(
            package='para_ai',
            executable=FindExecutable(name='python3'),
            arguments=[resize_node_path],
            name='resize_node',
            output='screen',
            parameters=[
                {'input_topic': 'carla/ego_vehicle/cam_br/image'},
                {'output_topic': 'para_ai/cam_br/image'},
            ],
        ),

        Node(
            package='para_ai',
            namespace='',
            executable=FindExecutable(name='python3'),
            arguments=[bev_node_path],
            name='bev_inference_node',
            output='screen',
            parameters=[
                {'model_path': '/home/paradigm/GitHub/igvc-2023-machine-learning/cross_view_transformers/logs/igvc_test/0522_162310/checkpoints/model-v1.ckpt'},
                {'calibration_file': calibration_file_path},
            ],
        ),
    ])
