#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios

class SafePointPublisher(Node):
    def __init__(self):
        super().__init__('safe_point_publisher')
        
        # Create publishers for each coordinate
        self.x_pub = self.create_publisher(String, '/x_coordinate', 10)
        self.y_pub = self.create_publisher(String, '/y_coordinate', 10)
        self.z_pub = self.create_publisher(String, '/z_coordinate', 10)
        
        # Predefined coordinate sets
        self.coordinate_sets = [
            "X:11, Y:12, Z:13",
            "X:21, Y:22, Z:23", 
            "X:31, Y:32, Z:33"
        ]
        self.current_set = 0
        
        # Configure terminal for non-blocking input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        # Create timer to publish continuously
        self.timer = self.create_timer(0.1, self.publish_coordinates)
        
        # Flag to track if Enter was pressed
        self.enter_pressed = False
        
    def publish_coordinates(self):
        """Publish coordinates based on current state"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            if key == '\n':  # Enter key pressed
                self.enter_pressed = True
                self.current_set = (self.current_set + 1) % len(self.coordinate_sets)
        
        if self.enter_pressed:
            # Publish current coordinate set
            msg = String()
            msg.data = self.coordinate_sets[self.current_set]
            self.x_pub.publish(msg)
            self.y_pub.publish(msg)
            self.z_pub.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")
        else:
            # Publish zeros
            zero_msg = String()
            zero_msg.data = "X:0, Y:0, Z:0"
            self.x_pub.publish(zero_msg)
            self.y_pub.publish(zero_msg)
            self.z_pub.publish(zero_msg)
    
    def destroy_node(self):
        """Clean up terminal settings before shutdown"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SafePointPublisher()
    
    print("Press Enter to publish coordinates (Ctrl+C to exit)...")
    print("Currently publishing zeros until Enter is pressed...")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()