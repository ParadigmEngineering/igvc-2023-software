
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class RPIGStreamer(Node):
    def __init__(self):
        super().__init__('rpi_gstreamer_node')
        ip = self.declare_parameter('server_ip', '127.0.0.1').value
        port = self.declare_parameter('server_port', '5000').value
        output_topic = self.declare_parameter('output_topic', '/output_image_topic').value
        

        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.bridge = CvBridge()

        gstreamer_pipeline = f"tcpclientsrc host={ip} port={port} ! multipartdemux ! jpegdec ! videoconvert ! appsink sync=false"

        # Open GStreamer video
        self.video = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        timer_period = 1/60 # seconds (60fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, cv_img = self.video.read()
        if not ret:
            return # self.get_logger().warning('No frame received')
        
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        resized_image = cv2.resize(cv_img, (480, 224))
        
        # Convert the resized OpenCV image to an image message
        img_msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='rgb8')

        # Set the frame_id and stamp
        img_msg.header.frame_id = 'base_link_cam_b'
        img_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the image
        self.publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)

    image_resizer = RPIGStreamer()
    try:
        rclpy.spin(image_resizer)
    except KeyboardInterrupt:
        pass

    image_resizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
