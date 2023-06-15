import rclpy
from rclpy.node import Node
# from can_interface import CanInterface
from geometry_msgs.msg import Twist
import logging
import os

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        # self.can_interface = CanInterface()

        self.left_id = self.get_id_left_motor_control_fwd()
        self.left_data = self.get_data_min_payload()
        self.prev_left_id = self.get_id_left_motor_control_fwd()
        self.prev_left_data = self.get_data_min_payload()

        self.right_id = self.get_id_right_motor_control_fwd()
        self.right_data = self.get_data_min_payload()
        self.prev_right_id = self.get_id_right_motor_control_fwd()
        self.prev_right_data = self.get_data_min_payload()

        self.state_id = self.get_id_right_motor_control_fwd()
        # self.state_data = self.can_interface.get_data_min_payload()
        self.prev_state_id = self.get_id_right_motor_control_fwd()
        # self.prev_state_data = self.can_interface.get_data_min_payload()

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        #self.subscription  # prevent unused variable warning
        self.logger = self.get_logger()
        self.logger.set_level(logging.INFO)
        self.logger.info('This is a MotorController information message')

    def listener_callback(self, msg):
        # self.logger.info('Getting a twist!')
        left_motor = msg.linear.x
        right_motor = msg.linear.y
        
        standby_request = msg.angular.x
        manual_request = msg.angular.y
        autonomous_request = msg.angular.z

        # 1.0 BASE Diameter
        # 0.2032 WHEEL RADIUS
        # WHEEL_BASE = 0.5  # replace with your robot's wheel base
        # WHEEL_RADIUS = 0.1  # replace with your robot's wheel radius

        # self.logger.info(f'Right motor value {left_motor}')
        # self.logger.info(f'Left motor value {right_motor}')
        # self.logger.info(f'Standby request value {standby_request}')
        # self.logger.info(f'Manual request value {manual_request}')
        # self.logger.info(f'Autonomous request value {autonomous_request}')

        # self.logger.info(f'Angular z value {linear_y}')
        # right_velocity = (2 * linear_x + angular_z )#* WHEEL_BASE) / (2 * WHEEL_RADIUS)
        # left_velocity = (2 * linear_x - angular_z )#* WHEEL_BASE) / (2 * WHEEL_RADIUS)
        
        self.set_motor_speeds(left_motor, right_motor)
        self.request_state_change(int(standby_request), int(manual_request), int(autonomous_request))
        # self.send_heartbeat()
    
    def round_to_nearest_increment(self, value, increment):
        return round(value / increment) * increment
    
    def lerp(self, t):
        """
        Linearly interpolates between v0 and v1 by t
        :param v0: start value
        :param v1: end value
        :param t: interpolation factor, between 0 and 1
        :return: interpolated value
        """

        V0 = 26
        V1 = 3649
        lerp_val = self.round_to_nearest_increment((1 - t) * V0 + t * V1, 250)
        if lerp_val >= V1:
            lerp_val = V1

        return lerp_val

    def set_motor_speeds(self, left, right):

        if left > 0.0:
            self.left_id = self.get_id_left_motor_control_fwd()
            hex_byte_array = self.int_to_hex_byte_array(int(self.lerp(abs(left))))
            self.left_data = list(" ".join(f"{i:02x}" for i in hex_byte_array).replace(" ", ""))
        elif left < 0.0:
            self.left_id = self.get_id_left_motor_control_rev()
            hex_byte_array = self.int_to_hex_byte_array(int(self.lerp(abs(left))))
            self.left_data = list(" ".join(f"{i:02x}" for i in hex_byte_array).replace(" ", ""))
        elif left == 0.0:
            self.left_id = self.prev_left_id
            self.left_data = self.get_data_min_payload()
        
        # self.logger.info(f'LEFT ID: {self.left_id}')

        left_id_changed = self.left_id != self.prev_left_id
        left_data_changed = self.left_data != self.prev_left_data

        if (left_id_changed or left_data_changed):
            hex_list = []
            for i in range(0, len(self.left_data), 2):
                hex_str = str(self.left_data[i]) + str(self.left_data[i+1])  # Join the pair of items
                hex_val = int(hex_str, 16)  # Convert the string to an integer using base 16
                hex_list.append(hex_val)
            # self.logger.info(f'ID: {self.left_id}')
            # self.logger.info(f'DATA: {hex_list}')
            self.send_can_message(self.left_id, hex_list)

        if right > 0.0:
            self.right_id = self.get_id_right_motor_control_fwd()
            hex_byte_array = self.int_to_hex_byte_array(int(self.lerp(abs(right))))
            self.right_data = list(" ".join(f"{i:02x}" for i in hex_byte_array).replace(" ", ""))
        elif right < 0.0:
            self.right_id = self.get_id_right_motor_control_rev()
            hex_byte_array = self.int_to_hex_byte_array(int(self.lerp(abs(right))))
            self.right_data = list(" ".join(f"{i:02x}" for i in hex_byte_array).replace(" ", ""))
        elif right == 0.0:
            self.right_id = self.prev_right_id
            self.right_data = self.get_data_min_payload()
        
        # self.logger.info(f'RIGHT ID: {self.right_id}')

        right_id_changed = self.right_id != self.prev_right_id
        right_data_changed = self.right_data != self.prev_right_data

        if (right_id_changed or right_data_changed):
            hex_list = []
            for i in range(0, len(self.right_data), 2):
                hex_str = str(self.right_data[i]) + str(self.right_data[i+1])  # Join the pair of items
                hex_val = int(hex_str, 16)  # Convert the string to an integer using base 16
                hex_list.append(hex_val)
            # self.logger.info(f'ID: {self.right_id}')
            # self.logger.info(f'DATA: {hex_list}')
            self.send_can_message(self.right_id, hex_list)

        self.prev_left_id = self.left_id
        self.prev_left_data = self.left_data

        self.prev_right_id = self.right_id
        self.prev_right_data = self.right_data

        pass

    def request_state_change(self, stdby_req, man_req, auto_req):
        if auto_req == 1:
            self.state_id = self.get_id_autonomous_control_request()
        elif man_req == 1:
            self.state_id = self.get_id_manual_control_request()
        elif stdby_req == 1:
            self.state_id = self.get_id_standby_request()
        
        if self.state_id != self.prev_state_id:
            self.send_can_message(self.state_id, self.get_data_min_payload())

        self.prev_state_id = self.state_id
        pass

    def send_heartbeat(self):
        self.left_id = self.get_id_heartbeat()
        self.left_data = self.get_data_min_payload()
        # self.logger.info(f'Logging ID: {self.left_id}, Data: {self.left_data}')
        self.send_can_message(self.left_id, self.left_data)
        pass
    def send_can_message(self, can_id, data):
        # message = can.Message(arbitration_id=can_id, data=data)
        # print(bus)
        # print(can_id)
        # print(data)
        # try:
        #     bus.send(message)
        #     print(f"Sent CAN message: ID={hex(can_id)}, Data={bytes(data).hex()}")
        # except can.CanError:
        #     print("Error sending CAN message")
        # Hack to fix on comp flight computer 
        data_str = ""
        for d in data:
            data_str += str(d)

        print(data_str)

        cmd_str = f"cansend can0 {can_id}#{data_str}"
        self.logger.info("cmd_str")
        print(cmd_str)

        os.system(cmd_str)

    def int_to_hex_byte_array(self, int_value):
        hex_byte_array = int_value.to_bytes(8, byteorder='big')
        return hex_byte_array

    # def int_to_bytearray(self, num):
    #     return struct.pack('i', num)
    
    def get_data_payload(self, freq):
        return self.int_to_hex_byte_array(int(freq))

    def get_id_standby_request(self):
        return 0x011

    def get_id_autonomous_control_request(self):
        return 0x012

    def get_id_manual_control_request(self):
        return 0x013

    def get_id_left_motor_control_fwd(self):
        return 0x021

    def get_id_left_motor_control_rev(self):
        return 0x022

    def get_id_right_motor_control_fwd(self):
        return 0x041

    def get_id_right_motor_control_rev(self):
        return 0x042
    
    def get_id_heartbeat(self):
        return 0x001

    def get_data_min_payload(self):
        return [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    def get_data_eigth_payload(self):
        return [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F]

    def get_data_quarter_payload(self):
        return [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF]
    
    def get_data_three_eigth_payload(self):
        return [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF]

    def get_data_half_payload(self):
        return [0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF]
    
    def get_data_five_eigth_payload(self):
        return [0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF]

    def get_data_three_quarter_payload(self):
        return [0x00, 0x00, 0x00, 0x00, 0x00,0xFF, 0xFF, 0xFF]
    
    def get_data_seven_eigth_payload(self):
        return [0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF]
    
    def get_data_full_payload(self):
        return [0x00, 0x00, 0x00, 0x00, 0xAF, 0xFF, 0xFF, 0xFF]

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    rclpy.spin(motor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()