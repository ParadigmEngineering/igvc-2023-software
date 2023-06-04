import cv2
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class JetsonGStreamer(Node):
    def __init__(self):
        super().__init__('my_video_node')
        ip = self.declare_parameter('server_ip', '127.0.0.1').value
        port = self.declare_parameter('server_port', '5000').value
        output_topics = [self.declare_parameter('output_topic_' + str(i), '/output_image_topic_' + str(i)).value for i in range(4)]

        self.bridge = CvBridge()
        
        self.publishers_ = [self.create_publisher(Image, output_topics[i], 10) for i in range(4)]

        gstreamer_pipeline = f"tcpclientsrc host={ip} port={port} ! multipartdemux ! jpegdec ! videoconvert ! appsink sync=false"

        # Open GStreamer video
        self.video = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        timer_period = 1/60 # seconds (60fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, cv_img = self.video.read()
        if not ret:
            #self.get_logger().warning('No frame received')
            return

        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

         # Split the image into four using numpy slicing
        height, width, _ = cv_img.shape
        half_height, half_width = height // 2, width // 2

        images = [cv_img[:half_height, :half_width], # top-left image
                  cv_img[:half_height, half_width:], # top-right image
                  cv_img[half_height:, :half_width], # bottom-left image
                  cv_img[half_height:, half_width:]  # bottom-right image
                  ]

        for i, img in enumerate(images):
            resized_image = cv2.resize(img, (480, 224))
            img_msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='rgb8')

            # Set the frame_id and stamp
            img_msg.header.frame_id = f'base_link_cam_{i}'
            img_msg.header.stamp = self.get_clock().now().to_msg()

            # Publish the image
            self.publishers_[i].publish(img_msg)

def main(args=None):
    rclpy.init(args=args)

    image_resizer = JetsonGStreamer()
    try:
        rclpy.spin(image_resizer)
    except KeyboardInterrupt:
        pass

    image_resizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
