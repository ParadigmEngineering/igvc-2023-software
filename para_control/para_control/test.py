import can
import time
import os

class CanInterface:
    def __init__(self, interface='can0'):
        self.interface = interface
        # self.bus = can.interface.Bus(channel=self.interface, bustype='socketcan')

    def send_can_message(self, can_id, data):
        # message = can.Message(arbitration_id=can_id, data=data)
        
        id_str = str(can_id)

        data_str = ""
        for d in data:
            data_str += str(d)
        

        print(data_str)
        print(id_str)

        cmd_str = f"cansend can0 {can_id}#{data_str}"
        print(cmd_str)

        os.system(cmd_str)

        # try:
        #     self.bus.send(message)
        #     print(f"Sent CAN message: ID={hex(can_id)}, Data={bytes(data).hex()}")
        # except can.CanError:
        #     print("Error sending CAN message")

def main():
    # Initialize CanInterface
    can_interface = CanInterface()

    # Prepare test message
    can_id ="011"
    data = ["00", "00", "00", "00", "11", "22", "33", "44"]

    # Send message every second
    while True:
        can_interface.send_can_message(can_id, data)
        time.sleep(1)

if __name__ == "__main__":
    main()