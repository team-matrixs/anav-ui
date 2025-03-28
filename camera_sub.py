#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import threading
import customtkinter as ctk
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage


class VideoSubscriber(Node):
    def __init__(self, image_label, screen_width):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.image_label = image_label
        self.screen_width = screen_width

    def image_callback(self, msg):
        # self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB

        # Convert OpenCV frame to PIL format
        img = PILImage.fromarray(frame)

        # Resize image based on screen width
        resized_img = img.resize((int(self.screen_width * 0.45), 750))

        # Convert to CTkImage
        ctk_img = ctk.CTkImage(light_image=resized_img, size=(
            int(self.screen_width * 0.45), 750))

        # Update UI on main thread
        self.image_label.configure(image=ctk_img)

        self.image_label.image = ctk_img  # Keep a reference
