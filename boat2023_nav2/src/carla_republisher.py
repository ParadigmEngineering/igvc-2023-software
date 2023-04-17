import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class CARLARepublisher(Node):

    def __init__(self):
        super().__init__('republisher')
        self.imu_subscription = self.create_subscription(
            Imu,
            '/carla/ego_vehicle/imu',
            self.imu_callback,
            10)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odom_callback,
            10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

    def imu_callback(self, msg):
        self.imu_publisher.publish(msg)

    def odom_callback(self, msg):
        self.odom_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    republisher = CARLARepublisher()

    rclpy.spin(republisher)

    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
