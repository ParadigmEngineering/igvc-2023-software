import cv2
import rclpy
import torch
import numpy as np

from rclpy.node import Node
from functools import partial
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cross_view_transformer.common import load_backbone


class BEVInferenceNode(Node):
    camera_names = ['f', 'fl', 'fr', 'b', 'bl', 'br']

    def __init__(self):
        super().__init__('bev_inference_node')

        self.model_path = self.declare_parameter('model_path').value
        self.calibration_file = self.declare_parameter('calibration_file').value

        self.target_fps = 30  # Set your desired FPS
        self.timer_period = 1.0 / self.target_fps
        self.timer = self.create_timer(self.timer_period, self.inference)

        self.init_network()
        self.init_bridge()
        self.init_calibration()
        self.init_subscribers()
        self.init_publisher()

    def init_network(self):
        self.network = load_backbone(self.model_path)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.network.to(self.device)
        self.network.eval()
        self.get_logger().info(f'Using device: {self.device}')

    def init_bridge(self):
        self.bridge = CvBridge()

    def init_calibration(self):
        with np.load(self.calibration_file, allow_pickle=True) as data:
            self.extrinsics = torch.from_numpy(data['aux'][()]['extrinsics'].astype('float32')).unsqueeze(0)
            self.intrinsics = torch.from_numpy(data['aux'][()]['intrinsics'].astype('float32')).unsqueeze(0)

    def init_subscribers(self):
        self.image_buffer = {}
        self.image_subscribers = {}

        for camera_name in self.camera_names:
            self.image_subscribers[camera_name] = self.create_subscription(
                Image,
                f'para_ai/cam_{camera_name}/image',
                partial(self.image_callback, camera_name),
                10,
                callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
            )

    def init_publisher(self):
        self.publisher = self.create_publisher(Image, 'para_ai/bev_prediction', 10)

    def image_callback(self, camera_name, msg):
        self.image_buffer[camera_name] = self.bridge.imgmsg_to_cv2(msg)

    def inference(self):
        batch = self.prepare_batch()

        if batch is None:
            return

        with torch.no_grad():
            batch = {k: v.to(self.device) if isinstance(v, torch.Tensor) else v for k, v in batch.items()}
            pred = self.network(batch)
            
            # Convert tensor to NumPy array and normalize to the range [0, 1]
            confidence_array = pred['center'].squeeze(0).squeeze(0).cpu().numpy()
            confidence_array = (confidence_array - confidence_array.min()) / (confidence_array.max() - confidence_array.min())

            # Convert the array to an 8-bit unsigned integer in the range [0, 255]
            confidence_array = (confidence_array * 255).astype(np.uint8)

            # Convert the array to an image message
            output_image = self.bridge.cv2_to_imgmsg(confidence_array, encoding="mono8")
            self.publisher.publish(output_image)


    def prepare_batch(self):
        if len(self.image_buffer) < 6:
            return None

        images = torch.empty(size=(6, 3, 224, 480))
        for i, camera_name in enumerate(self.camera_names):
            images[i] = torch.from_numpy(cv2.cvtColor(self.image_buffer[camera_name], cv2.COLOR_BGR2RGB)).permute(2, 0, 1)

        batch = {
            'extrinsics': self.extrinsics,
            'intrinsics': self.intrinsics,
            'image': images.unsqueeze(0),
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
