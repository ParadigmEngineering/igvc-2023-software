import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('para_ai')
    bev_node_path = os.path.join(pkg_share, 'para_ai', 'bev_inference_node.py')
    calibration_file_path = os.path.join(pkg_share, 'config', 'camera_calibration.yaml')

    return LaunchDescription([
        Node(
            package='para_ai',
            namespace='',
            executable=FindExecutable(name='python3'),
            arguments=[bev_node_path],
            name='bev_inference_node',
            output='screen',
            parameters=[
                {'model_path': '/home/paradigm/GitHub/igvc-2023-machine-learning/cross_view_transformers/logs/igvc_test/0502_224206/checkpoints/model.ckpt'},
                {'calibration_file': calibration_file_path},
            ],
        ),
    ])
