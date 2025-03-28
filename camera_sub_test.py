#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 0)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("Receiving video frame")
        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("ROS 2 Camera Feed", frame)
        cv2.waitKey(1)  # Required for OpenCV to refresh image window


def main(args=None):
    rclpy.init(args=args)
    node = VideoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # Close OpenCV windows when shutting down


if __name__ == '__main__':
    main()
