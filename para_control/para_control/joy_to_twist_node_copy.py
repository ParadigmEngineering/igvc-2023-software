import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from can_interface import CanInterface
import logging

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')

        self.can_interface = CanInterface()
        self.state_id = self.can_interface.get_id_standby_request()
        self.prev_state_id = self.can_interface.get_id_standby_request()

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
    
    def motor_commands_to_twist(self, left_motor_magnitude, right_motor_magnitude):
        # Initialize a Twist message
        twist = Twist()

        # Max velocity is 5 km/h
        MAX_VELOCITY = 5

        # The linear velocity (forward speed) is the average of the two motor commands
        twist.linear.x = ((left_motor_magnitude * MAX_VELOCITY) + (right_motor_magnitude * MAX_VELOCITY)) / 2

        # The angular velocity (rotational speed) is the difference between the two motor commands
        twist.angular.z = ((right_motor_magnitude * MAX_VELOCITY) - ((left_motor_magnitude * MAX_VELOCITY))) / 2

        return twist

    def joy_callback(self, msg):
        # create a new Twist message
        left_motor_magnitude = msg.axes[1]
        right_motor_magnitude = msg.axes[4]
        twist = self.motor_commands_to_twist(left_motor_magnitude, right_motor_magnitude)

        self.logger.info(f'Linear x: {twist.linear.x}')
        self.logger.info(f'Angular z: {twist.angular.z}')
        # self.logger.info(f'standby request X is {twist.angular.x}')
        # self.logger.info(f'standby request Y is {twist.angular.y}')
        # self.logger.info(f'standby request B is {twist.angular.z}')

        # Publish the Twist message
        self.publisher_.publish(twist)

        R1 = msg.buttons[5]

        # Map standby request to be R1 & X button
        standby_request = float(R1 and msg.buttons[2])

        #Map manual request to be R1 & Y button
        manual_request = float(R1 and msg.buttons[3])

        # Map autonomous request to be R1 & B button
        autonomous_request = float(R1 and msg.buttons[1])

        # Publish CAN State Change messages
        self.request_state_change(int(standby_request), int(manual_request), int(autonomous_request))

    def request_state_change(self, stdby_req, man_req, auto_req):
        if auto_req == 1:
            self.state_id = self.can_interface.get_id_autonomous_control_request()
        elif man_req == 1:
            self.state_id = self.can_interface.get_id_manual_control_request()
        elif stdby_req == 1:
            self.state_id = self.can_interface.get_id_standby_request()
        
        if self.state_id != self.prev_state_id:
            self.can_interface.send_can_message(self.can_interface.bus, self.state_id, self.can_interface.get_data_min_payload())

        self.prev_state_id = self.state_id
        pass

def main(args=None):
    rclpy.init(args=args)

    joy_to_twist = JoyToTwist()

    rclpy.spin(joy_to_twist)

    # Destroy the node explicitly
    joy_to_twist.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
