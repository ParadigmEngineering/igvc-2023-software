import cv2
import rclpy
import torch
import numpy as np
import torchvision.transforms as transforms

from rclpy.node import Node
from functools import partial
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cross_view_transformer.common import load_backbone
from cross_view_transformer.visualizations.common import BaseViz


class BEVInferenceNode(Node):
    batch = None
    camera_names = ['f', 'fl', 'fr', 'b', 'bl', 'br']
    transform = transforms.Compose([transforms.ToTensor()])

    def __init__(self):
        super().__init__('bev_inference_node')

        self.model_path = self.declare_parameter('model_path').value
        self.calibration_file = self.declare_parameter('calibration_file').value
        self.threshold = self.declare_parameter('threshold').value

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

        self.viz = BaseViz()

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
        self.prepare_batch()

    def inference(self):
        if self.batch is None:
            return

        with torch.no_grad():
            curr_batch = {k: v.to(self.device) if isinstance(v, torch.Tensor) else v for k, v in self.batch.items()}
            pred = self.network(curr_batch)

            # For debug 6 camera view:
            # curr_batch['bev'] = pred['bev']
            # visualization = np.vstack(self.viz.visualize(batch=curr_batch, pred=pred))

            pred = pred['bev'][0].sigmoid()
            pred = pred.cpu().numpy().transpose(1, 2, 0)
            pred = (pred > self.threshold).astype(np.float32)
            
            # Convert the array to an image message
            output_image = self.bridge.cv2_to_imgmsg(pred)
            self.publisher.publish(output_image)


    def prepare_batch(self):
        if len(self.image_buffer) < 6:
            return

        images = torch.empty(size=(6, 3, 224, 480))
        for i, camera_name in enumerate(self.camera_names):
            images[i] = self.transform(self.image_buffer[camera_name])

        self.batch = {
            'extrinsics': self.extrinsics,
            'intrinsics': self.intrinsics,
            'image': images.unsqueeze(0),
        }

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
