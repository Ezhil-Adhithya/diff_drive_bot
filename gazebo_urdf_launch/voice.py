import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Keyboard Teleop Node Started. Use WASD keys to move. Press 'Q' to stop.")

    def get_key(self):
        """ Read a single key press from the keyboard """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':  # Forward
                twist.linear.x = -0.5
                twist.angular.z = 0.0
            elif key == 's':  # Backward
                twist.linear.x = 0.5
                twist.angular.z = 0.0
            elif key == 'a':  # Turn Left
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            elif key == 'd':  # Turn Right
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            elif key == 'q':  # Stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == '\x03':  # CTRL+C
                break
            
            self.publisher_.publish(twist)
            self.get_logger().info(f"Publishing: Linear={twist.linear.x}, Angular={twist.angular.z}")

def main():
    rclpy.init()
    teleop_node = KeyboardTeleop()
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
