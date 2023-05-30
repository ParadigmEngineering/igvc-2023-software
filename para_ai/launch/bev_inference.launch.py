import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('para_ai')

    resize_node_path = os.path.join(pkg_share, 'para_ai', 'resize_node.py')
    rpi_gstreamer_node_path = os.path.join(pkg_share, 'para_ai', 'rpi_gstreamer_node.py')
    jetson_gstreamer_node_path = os.path.join(pkg_share, 'para_ai', 'jetson_gstreamer_node.py')
    
    bev_node_path = os.path.join(pkg_share, 'para_ai', 'bev_inference_node.py')
    calibration_file_path = os.path.join(pkg_share, 'config', 'camera_calibration.npz')

    return LaunchDescription([

        # RPi Back Camera
        Node(
            package='para_ai',
            executable=FindExecutable(name='python3'),
            arguments=[jetson_gstreamer_node_path],
            name='jetson_gstreamer_node',
            output='screen',
            parameters=[
                {'server_ip': '192.168.0.132'},
                {'server_port': '5002'},
                {'output_topic_0': 'para_ai/cam_fl/image'},
                {'output_topic_1': 'para_ai/cam_fr/image'},
                {'output_topic_2': 'para_ai/cam_bl/image'},
                {'output_topic_3': 'para_ai/cam_br/image'},
            ],
        ),

        # RPi Back Camera
        Node(
            package='para_ai',
            executable=FindExecutable(name='python3'),
            arguments=[rpi_gstreamer_node_path],
            name='rpi_gstreamer_node',
            output='screen',
            parameters=[
                {'server_ip': '192.168.0.163'},
                {'server_port': '5000'},
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
                {'input_topic': 'zed2/zed_node/left/image_rect_color'},
                {'output_topic': 'para_ai/cam_f/image'},
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
                {'model_path': '/home/paradigm/GitHub/igvc-2023-machine-learning/prod_models/model.ckpt'},
                {'calibration_file': calibration_file_path},
                {'threshold': 0.5},
            ],
        ),
    ])
