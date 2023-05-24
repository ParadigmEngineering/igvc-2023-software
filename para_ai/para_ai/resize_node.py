import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class ImageResizer(Node):
    def __init__(self):
        super().__init__('image_resizer')

        self.bridge = CvBridge()
        input_topic = self.declare_parameter('input_topic', '/input_image_topic').value
        output_topic = self.declare_parameter('output_topic', '/output_image_topic').value
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, output_topic, 10)

    def listener_callback(self, msg):
        # Convert the image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Resize the image
        resized_image = cv2.resize(cv_image, (480, 224))

        # Convert the resized OpenCV image to an image message
        output_msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')

        # Set the frame_id and stamp
        output_msg.header.frame_id = msg.header.frame_id
        output_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the resized image
        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)

    image_resizer = ImageResizer()
    try:
        rclpy.spin(image_resizer)
    except KeyboardInterrupt:
        pass

    image_resizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
