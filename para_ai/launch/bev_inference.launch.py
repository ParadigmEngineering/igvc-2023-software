from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            namespace='',
            executable='bev_inference_node',
            name='bev_inference_node',
            output='screen',
            parameters=[
                {'model_path': 'path/to/your/model/file.ckpt'},
                {'calibration_file': 'path/to/your/package/config/camera_calibration.yaml'},
            ],
        ),
    ])
