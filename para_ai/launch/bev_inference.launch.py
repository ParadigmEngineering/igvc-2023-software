import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('para_ai')
    resize_node_path = os.path.join(pkg_share, 'para_ai', 'resize_node.py')
    gstreamer_node_path = os.path.join(pkg_share, 'para_ai', 'gstreamer_node.py')
    bev_node_path = os.path.join(pkg_share, 'para_ai', 'bev_inference_node.py')
    calibration_file_path = os.path.join(pkg_share, 'config', 'camera_calibration.npz')

    return LaunchDescription([
        Node(
            package='para_ai',
            executable=FindExecutable(name='python3'),
            arguments=[gstreamer_node_path],
            name='resize_node',
            output='screen',
            parameters=[
                {'server_ip': '192.168.0.163'},
            ],
        ),

        # Node(
        #     package='para_ai',
        #     namespace='',
        #     executable=FindExecutable(name='python3'),
        #     arguments=[bev_node_path],
        #     name='bev_inference_node',
        #     output='screen',
        #     parameters=[
        #         {'model_path': '/home/paradigm/GitHub/igvc-2023-machine-learning/prod_models/model.ckpt'},
        #         {'calibration_file': calibration_file_path},
        #         {'threshold': 0.5},
        #     ],
        # ),
    ])
