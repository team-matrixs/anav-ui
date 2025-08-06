# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import random
# import time

# class SafePointPublisher(Node):
#     def __init__(self):
#         super().__init__('safe_point_publisher')
        
#         # Create publisher with String message type
#         self.publisher = self.create_publisher(String, '/text_topic', 10)
        
#         # Timer to publish points every 2 seconds
#         self.timer = self.create_timer(2.0, self.publish_safe_points)
        
#         # Sample safe points data
#         self.safe_points = [
#             ("1", "01.99", "01.77"),  # Point 1
#             ("2", "02.01", "01.60"),   # Point 2
#             ("3", "03.36", "00.92")    # Point 3
#         ]
        
#         self.get_logger().info("Safe Point Publisher Started")

#     def publish_safe_points(self):
#         """Publish safe points one by one in sequence"""
#         for point in self.safe_points:
#             # Format: "N : X:XX.XX Y:YY.YY"
#             point_num, x, y = point
#             msg = String()
#             msg.data = f"{point_num} : X:{x} Y:{y}"
            
#             # Publish the message
#             self.publisher.publish(msg)
#             self.get_logger().info(f"Published: {msg.data}")
#             time.sleep(5)
            
#             # Add some random variation to coordinates if desired
#             # self.safe_points = [
#             #     (1, random.uniform(0, 2), random.uniform(0, 2)),
#             #     (2, random.uniform(0, 2), random.uniform(0, 2)),
#             #     (3, random.uniform(0, 2), random.uniform(0, 2))
#             # ]

# def main(args=None):
#     rclpy.init(args=args)
#     publisher = SafePointPublisher()
    
#     try:
#         rclpy.spin(publisher)
#     except KeyboardInterrupt:
#         publisher.get_logger().info("Shutting down publisher...")
#     finally:
#         publisher.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SafeSpotPublisher(Node):
    def __init__(self):
        super().__init__('safe_spot_publisher')
        self.publisher_ = self.create_publisher(String, '/text_topic', 10)
        self.timer = self.create_timer(2.0, self.publish_safe_spots)  # Publish every 2 seconds
        self.coordinate_counter = 1
        self.safe_spots = [
            (1.5, 2.0),  # First safe spot (x, y)
            (3.0, 1.5),  # Second safe spot
            (2.5, 3.0)   # Third safe spot
        ]
        self.current_spot = 0
        
    def publish_safe_spots(self):
        if self.current_spot < len(self.safe_spots):
            msg = String()
            x, y = self.safe_spots[self.current_spot]
            msg.data = f"{self.coordinate_counter} : X:{x:05.2f} Y:{y:05.2f}"
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published safe spot: {msg.data}')
            
            self.coordinate_counter += 1
            self.current_spot += 1
        else:
            self.get_logger().info('All safe spots published')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    publisher = SafeSpotPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()