import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pynput

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.keyboard_listener = pynput.keyboard.Listener(on_press=self.on_keypress)
        self.keyboard_listener.start()

    def on_keypress(self, key):
        twist = Twist()
        linear_speed = 0.5  # adjust to your robot's capabilities
        angular_speed = 0.5  # adjust to your robot's capabilities

        if key == pynput.keyboard.Key.up or key == pynput.keyboard.KeyCode.from_char('w'):
            twist.linear.x = linear_speed
        elif key == pynput.keyboard.Key.down or key == pynput.keyboard.KeyCode.from_char('s'):
            twist.linear.x = -linear_speed
        elif key == pynput.keyboard.Key.left or key == pynput.keyboard.KeyCode.from_char('a'):
            twist.angular.z = angular_speed
        elif key == pynput.keyboard.Key.right or key == pynput.keyboard.KeyCode.from_char('d'):
            twist.angular.z = -angular_speed

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    teleop_publisher = TeleopPublisher()

    rclpy.spin(teleop_publisher)

    teleop_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
