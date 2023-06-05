import cv2
import rclpy
import torch
import numpy as np
import matplotlib.pyplot as plt
import torchvision.transforms as transforms

from rclpy.node import Node
from functools import partial
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cross_view_transformer.common import load_backbone
from cross_view_transformer.visualizations.common import BaseViz


class BEVInferenceNode(Node):
    camera_names = ['f', 'fl', 'fr', 'b', 'bl', 'br']
    camera_name_to_idx = {name: idx for idx, name in enumerate(camera_names)}

    batch = None
    transform = transforms.Compose([transforms.ToTensor()])

    def __init__(self):
        super().__init__('bev_inference_node')

        self.model_path = self.declare_parameter('model_path').value
        self.calibration_file = self.declare_parameter('calibration_file').value
        self.threshold = self.declare_parameter('threshold').value
        self.output_topic = self.declare_parameter('output_topic').value

        self.target_fps = 30  # Set desired FPS
        self.timer_period = 1.0 / self.target_fps
        self.timer = self.create_timer(self.timer_period, self.inference)

        self.init_network()
        self.init_bridge()
        self.init_batch()
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

    def init_batch(self):
        with np.load(self.calibration_file, allow_pickle=True) as data:
            self.extrinsics = torch.from_numpy(data['aux'][()]['extrinsics'].astype('float32')).unsqueeze(0)
            self.intrinsics = torch.from_numpy(data['aux'][()]['intrinsics'].astype('float32')).unsqueeze(0)
        
        images = torch.zeros((1, 6, 3, 224, 480), device=self.device)

        self.batch = {
            'extrinsics': self.extrinsics.to(self.device),
            'intrinsics': self.intrinsics.to(self.device),
            'image': images,
        }

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
        self.publisher = self.create_publisher(Image, self.output_topic, 10)

    def image_callback(self, camera_name, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        tensor_image = self.transform(cv_image).to(self.device)
        self.batch['image'][0, self.camera_name_to_idx[camera_name]] = tensor_image

    def inference(self):
        if self.batch is None:
            return

        with torch.no_grad():
            pred = self.network(self.batch)
            pred_bev = pred['bev'][0]

            # # Transfer pred to CPU and apply sigmoid, and thresholding, also convert to numpy
            # occupancy_grid = (torch.sigmoid(pred_bev).cpu().numpy() > self.threshold).astype(np.float32)

            # # Transpose and convert the array to an image message
            # output_image = self.bridge.cv2_to_imgmsg(occupancy_grid.transpose(1, 2, 0), encoding='bgr8')
            # self.publisher.publish(output_image)
            # Transfer pred to CPU and apply sigmoid, and thresholding, also convert to numpy
            occupancy_grid = (torch.sigmoid(pred_bev).cpu().numpy() > self.threshold).astype(np.float32)
            occupancy_grid = np.squeeze(occupancy_grid)
            occupancy_grid *= 255
            
            rgb_image = np.stack([occupancy_grid]*3, axis=2).astype(np.uint8)  # Replicate to R, G, and B channels
            output_image = self.bridge.cv2_to_imgmsg(rgb_image, "rgb8")
            
            self.publisher.publish(output_image)

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
