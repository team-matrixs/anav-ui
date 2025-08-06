#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class SafePointPublisher(Node):
    def __init__(self):
        super().__init__('safe_point_publisher')
        
        # Create publisher with String message type
        self.publisher = self.create_publisher(String, '/text_topic', 10)
        
        # Timer to publish points every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_safe_points)
        
        # Sample safe points data
        self.safe_points = [
            (1, 0.99, 1.77),  # Point 1
            (2, 0.01, 1.60),   # Point 2
            (3, 0.36, 0.92)    # Point 3
        ]
        
        self.get_logger().info("Safe Point Publisher Started")

    def publish_safe_points(self):
        """Publish safe points one by one in sequence"""
        for point in self.safe_points:
            # Format: "N : X:XX.XX Y:YY.YY"
            point_num, x, y = point
            msg = String()
            msg.data = f"{point_num} : X:{x:.2f} Y:{y:.2f}"
            
            # Publish the message
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")
            time.sleep(2)
            
            # Add some random variation to coordinates if desired
            # self.safe_points = [
            #     (1, random.uniform(0, 2), random.uniform(0, 2)),
            #     (2, random.uniform(0, 2), random.uniform(0, 2)),
            #     (3, random.uniform(0, 2), random.uniform(0, 2))
            # ]

def main(args=None):
    rclpy.init(args=args)
    publisher = SafePointPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info("Shutting down publisher...")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()