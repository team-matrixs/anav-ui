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
        label = ["x", "y", "z"]
        c1 = print("enter 1 : ")
        for i in range(0, 3):
            c1 = f"{label[i]}:"+input(f"{label[i]}: ")
        c2 = print("enter 2: ")
        for i in range(0, 3):
            c2 = f"{label[i]}:"+input(f"{label[i]}: ")
        c3 = print("enter 3: ")
        for i in range(0, 3):
            c3 = f"{label[i]}:"+input(f"{label[i]}: ")

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
            msg1 = String()
            msg1.data = self.coordinate_sets[0]
            msg2 = String()
            msg2.data = self.coordinate_sets[1]
            msg3 = String()
            msg3.data = self.coordinate_sets[2]
            self.x_pub.publish(msg1)
            self.y_pub.publish(msg2)
            self.z_pub.publish(msg3)
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