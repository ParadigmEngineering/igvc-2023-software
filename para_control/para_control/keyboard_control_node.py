import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class KeyboardController(Node):
    def __init__(self):
        super().__init__('image_resizer')
        print('Hello Paradigm')


def main(args=None):
    rclpy.init(args=args)

    image_resizer = KeyboardController()
    try:
        rclpy.spin(image_resizer)
    except KeyboardInterrupt:
        pass

    image_resizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
