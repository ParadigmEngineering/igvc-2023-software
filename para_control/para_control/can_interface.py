"""
CAN interface for the keyboard controller to send CAN messages.
Syncronizes message ids and payloads to what is expected on the node.

Example CAN transmission:

can_id = 0x021
data = [0xDE, 0xAD, 0xBE, 0xEF]
send_can_message(bus, can_id, data)
"""

import can
import os
import logging

class CanInterface:
    def __init__(self):
        self.interface = 'can0'
        self.logger = self.get_logger()
        self.logger.set_level(logging.INFO)
        self.logger.info('This is a joy_to_twist_node information message')
        # self.bus = can.interface.Bus(channel=self.interface, bustype='socketcan')

    def send_can_message(self, bus, can_id, data):
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
