import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped, Pose

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
        self.odom_publisher = self.create_publisher(Odometry, '/odometry', 10)

        self.gnss_subscription = self.create_subscription(
            NavSatFix,
            '/carla/ego_vehicle/gnss',
            self.gnss_callback,
            10)
        self.gnss_publisher = self.create_publisher(NavSatFix, '/gnss', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.initial_pose = Pose()
        self.initial_pose.position.x = -33.4
        self.initial_pose.position.y = 47.4
        self.initial_pose.position.z = 0.0

    def imu_callback(self, msg: Odometry):
        # Republish the IMU data
        msg.header.frame_id = 'imu_link'
        self.imu_publisher.publish(msg)

        # Broadcast the IMU transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        
        # TODO: Set the transform values based on the URDF file
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = 0.0, 0.0, 0.0
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = 0.0, 0.0, 0.0, 1.0

        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry):
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose

       # Subtract initial position from the current position
        msg.pose.pose.position.x -= self.initial_pose.position.x
        msg.pose.pose.position.y -= self.initial_pose.position.y
        msg.pose.pose.position.z -= self.initial_pose.position.z

        # Publish the odometry message
        msg.header.frame_id = 'odom'
        self.odom_publisher.publish(msg)

        # Broadcast the transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def gnss_callback(self, msg: NavSatFix):
        msg.header.frame_id = 'gnss_link'
        self.gnss_publisher.publish(msg)

        # Broadcast the GNSS transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'gnss_link'

        # TODO: Set the transform values based on the URDF file
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = 0.0, 0.0, 0.0
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = 0.0, 0.0, 0.0, 1.0

        self.tf_broadcaster.sendTransform(t)




def main(args=None):
    rclpy.init(args=args)

    republisher = CARLARepublisher()

    rclpy.spin(republisher)

    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
