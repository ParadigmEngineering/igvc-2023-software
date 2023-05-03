import rclpy
from rclpy.node import Node

import torch
import numpy as np
import cv2
import yaml

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from cross_view_transformer.common import load_backbone


class BEVInferenceNode(Node):
    def __init__(self):
        super().__init__('bev_inference_node')

        self.model_path = self.declare_parameter('model_path').value
        self.calibration_file = self.declare_parameter('calibration_file').value

        self.network = load_backbone(self.model_path)

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.network.to(self.device)
        self.network.eval()

        self.bridge = CvBridge()

        self.image_buffer = {}
        self.image_subscribers = {}
        camera_names = ['f', 'fl', 'fr', 'b', 'br', 'bl']

        for camera_name in camera_names:
            self.image_subscribers[camera_name] = self.create_subscription(Image, f'/{camera_name}', self.image_callback, 10, callback_group=rclpy.callback_groups.ReentrantCallbackGroup(), callback_args=camera_name)

        self.publisher = self.create_publisher(Image, '/output_image_topic', 10)

        with open(self.calibration_file, 'r') as f:
            calibration_data = yaml.safe_load(f)

        self.extrinsics = torch.tensor(calibration_data['extrinsics']).unsqueeze(0)
        self.intrinsics = torch.tensor(calibration_data['intrinsics']).unsqueeze(0)

    def image_callback(self, msg, camera_name):
        self.image_buffer[camera_name] = self.bridge.imgmsg_to_cv2(msg)

        if len(self.image_buffer) == 6:
            batch = self.prepare_batch()

            with torch.no_grad():
                batch = {k: v.to(self.device) if isinstance(v, torch.Tensor) else v for k, v in batch.items()}
                pred = self.network(batch)

                visualization = np.vstack(viz(batch=batch, pred=pred))
                output_image = self.bridge.cv2_to_imgmsg(visualization, encoding="bgr8")

                self.publisher.publish(output_image)

    def prepare_batch(self):
        image_tensors = [torch.from_numpy(cv2.cvtColor(self.image_buffer[camera_name], cv2.COLOR_BGR2RGB)).permute(2, 0, 1) for camera_name in self.image_buffer]
        image_batch = torch.stack(image_tensors).unsqueeze(0)

        batch = {
            'extrinsics': self.extrinsics,
            'intrinsics': self.intrinsics,
            'image': image_batch
        }

        return batch

def main(args=None):
    rclpy.init(args=args)

    node = BEVInferenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
