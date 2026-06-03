#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that provides manual keyboard control of the RoboBoat using the WASD keys to produce
#  per-thruster Float64 setpoints. It initializes thrusters to a known safe state on startup and
#  logs control actions during operation.
# ----------------------------------------------------------------------------------------------- #

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import select
import sys
import termios
import tty

STARTUP_MSG = "Manual control of the roboboat via the WASD keys has been successfully initiated."

class ThrusterControl(Node):

    def __init__(self):
        super().__init__('manual_control')
        
        self.left_thruster_topic = '/roboboat/thrusters/left/thrust'
        self.right_thruster_topic = '/roboboat/thrusters/right/thrust'
        
        self.left_thruster_pub = self.create_publisher(Float64, self.left_thruster_topic, 10)
        self.right_thruster_pub = self.create_publisher(Float64, self.right_thruster_topic, 10)
        
        self.stop_thrusters()

        self.terminal_settings = None
        self.keyboard_enabled = sys.stdin.isatty()
        if self.keyboard_enabled:
            self.terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.create_timer(0.05, self.poll_keyboard)
        else:
            self.get_logger().warning(
                "stdin is not a TTY; manual keyboard control is disabled for this process."
            )

        if not getattr(self, '_startup_message_printed', False):
            print(STARTUP_MSG)
            print("Controls: W forward, A turn left, D turn right, S stop, Q quit")
            self._startup_message_printed = True

    def start_thrusters(self):
        left_thruster_power = 10.0
        right_thruster_power = 10.0

        self.left_thruster_pub.publish(Float64(data=left_thruster_power))
        self.right_thruster_pub.publish(Float64(data=right_thruster_power))

        self.get_logger().info("Thrusters activated.")

    def stop_thrusters(self):
        left_thruster_power = 0.0  
        right_thruster_power = 0.0  
        
        self.left_thruster_pub.publish(Float64(data=left_thruster_power))
        self.right_thruster_pub.publish(Float64(data=right_thruster_power))
        
        self.get_logger().info("Thrusters deactivated.")

    def turn_left(self):
        left_thruster_power = -10.0  
        right_thruster_power = 10.0
        
        self.left_thruster_pub.publish(Float64(data=left_thruster_power))
        self.right_thruster_pub.publish(Float64(data=right_thruster_power))
        
        self.get_logger().info("Turning left.")

    def turn_right(self):
        left_thruster_power = 10.0 
        right_thruster_power = -10.0
        
        self.left_thruster_pub.publish(Float64(data=left_thruster_power))
        self.right_thruster_pub.publish(Float64(data=right_thruster_power))
        
        self.get_logger().info("Turning right.")

    def poll_keyboard(self):
        if not self.keyboard_enabled:
            return

        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return

        key = sys.stdin.read(1).lower()
        if key == 'w':
            self.start_thrusters()
        elif key == 's':
            self.stop_thrusters()
        elif key == 'a':
            self.turn_left()
        elif key == 'd':
            self.turn_right()
        elif key == 'q':
            self.stop_thrusters()
            raise KeyboardInterrupt

    def restore_terminal(self):
        if self.terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
