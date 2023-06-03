import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import logging

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')

        # Create subscriber for Joy messages
        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            qos)
        self.subscription  # prevent unused variable warning

        # Create publisher for Twist messages
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos)

        self.logger = self.get_logger()
        self.logger.set_level(logging.INFO)
        self.logger.info('This is a joy_to_twist_node information message')

    def joy_callback(self, msg):
        # create a new Twist message
        twist = Twist()

        # Map left joystick value to linear.x
        twist.linear.x = msg.axes[1]

        # Map right joystic value to linear.y
        twist.linear.y = msg.axes[4]

        R1 = msg.buttons[5]

        # Map standby request to be R1 & X button
        twist.angular.x = float(R1 and msg.buttons[2])

        #Map manual request to be R1 & Y button
        twist.angular.y = float(R1 and msg.buttons[3])

        # Map autonomous request to be R1 & B button
        twist.angular.z = float(R1 and msg.buttons[1])

        # self.logger.info(f'left joystick is set to {twist.linear.x}')
        # self.logger.info(f'right joystick is set to {twist.linear.y}')
        # self.logger.info(f'standby request X is {twist.angular.x}')
        # self.logger.info(f'standby request Y is {twist.angular.y}')
        # self.logger.info(f'standby request B is {twist.angular.z}')

        # Publish the Twist message
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    joy_to_twist = JoyToTwist()

    rclpy.spin(joy_to_twist)

    # Destroy the node explicitly
    joy_to_twist.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
