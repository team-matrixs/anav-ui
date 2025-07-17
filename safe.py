#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TextPublisher(Node):
    def __init__(self):
        super().__init__('text_publisher')
        self.publisher_ = self.create_publisher(String, 'text_topic', 10)
        timer_period = 1.0  # seconds (1 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = String()
        msg.data = "1 : X:20.04 Y:03.40 \n2 : X:50.05 Y:00.02\n3 : X:10.02 Y:52.04"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing:\n%s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    text_publisher = TextPublisher()
    
    try:
        rclpy.spin(text_publisher)
    except KeyboardInterrupt:
        pass
    
    text_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()