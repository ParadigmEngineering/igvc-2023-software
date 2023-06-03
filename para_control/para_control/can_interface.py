"""
CAN interface for the keyboard controller to send CAN messages.
Syncronizes message ids and payloads to what is expected on the node.

Example CAN transmission:

can_id = 0x021
data = [0xDE, 0xAD, 0xBE, 0xEF]
send_can_message(bus, can_id, data)
"""

import can

class CanInterface:
    def __init__(self):
        self.interface = 'can0'
        self.bus = can.interface.Bus(channel=self.interface, bustype='socketcan')

    def send_can_message(self, bus, can_id, data):
        message = can.Message(arbitration_id=can_id, data=data)
        # try:
        bus.send(message)
        print(f"Sent CAN message: ID={hex(can_id)}, Data={bytes(data).hex()}")
        # except can.CanError:
        #     print("Error sending CAN message")

    def int_to_hex_byte_array(self, int_value):
        hex_byte_array = int_value.to_bytes(4, byteorder='big')
        return hex_byte_array

    # def int_to_bytearray(self, num):
    #     return struct.pack('i', num)
    
    def get_data_payload(self, freq):
        return self.int_to_hex_byte_array(int(freq))

    def get_id_standby_request(self):
        return 0x11

    def get_id_autonomous_control_request(self):
        return 0x12

    def get_id_manual_control_request(self):
        return 0x13

    def get_id_left_motor_control_fwd(self):
        return 0x21

    def get_id_left_motor_control_rev(self):
        return 0x22

    def get_id_right_motor_control_fwd(self):
        return 0x41

    def get_id_right_motor_control_rev(self):
        return 0x42
    
    def get_id_heartbeat(self):
        return 0x01

    def get_data_min_payload(self):
        return [0x00, 0x00, 0x00, 0x00]
    
    def get_data_eigth_payload(self):
        return [0x00, 0x00, 0x00, 0x0F]

    def get_data_quarter_payload(self):
        return [0x00, 0x00, 0x00, 0xFF]
    
    def get_data_three_eigth_payload(self):
        return [0x00, 0x00, 0x0F, 0xFF]

    def get_data_half_payload(self):
        return [0x00, 0x00, 0xFF, 0xFF]
    
    def get_data_five_eigth_payload(self):
        return [0x00, 0x0F, 0xFF, 0xFF]

    def get_data_three_quarter_payload(self):
        return [0x00, 0xFF, 0xFF, 0xFF]
    
    def get_data_seven_eigth_payload(self):
        return [0x0F, 0xFF, 0xFF, 0xFF]
    
    def get_data_full_payload(self):
        return [0xAF, 0xFF, 0xFF, 0xFF]
