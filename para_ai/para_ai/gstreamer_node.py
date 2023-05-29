
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class GStreamer(Node):
    def __init__(self):
        super().__init__('my_video_node')
        self.declare_parameter('server_ip', '127.0.0.1')
        output_topic = self.declare_parameter('output_topic', '/output_image_topic').value
        

        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.bridge = CvBridge()

        server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        gstreamer_pipeline = f"tcpclientsrc host={server_ip} port=500 ! multipartdemux ! jpegdec ! videoconvert ! appsink sync=false"

        # Open GStreamer video
        self.video = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        timer_period = 1/60 # seconds (60fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, cv_img = self.video.read()
        if not ret:
            self.get_logger().warning('No frame received')
            return
        
        # Convert the image from BGR to RGB and then to a ROS image message
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        img_msg = self.bridge.cv2_to_imgmsg(cv_img, "rgb8")
        
        # Publish the image
        self.publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)

    image_resizer = GStreamer()
    try:
        rclpy.spin(image_resizer)
    except KeyboardInterrupt:
        pass

    image_resizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
