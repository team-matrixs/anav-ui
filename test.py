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
    def __init__(self, image_label, screen_width, root):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 0)
        self.bridge = CvBridge()
        self.image_label = image_label
        self.screen_width = screen_width
        self.root = root  # Store reference to Tkinter root for safe UI updates

    def image_callback(self, msg):
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

        # Schedule UI update in the main thread
        self.root.after(0, self.update_ui, ctk_img)

    def update_ui(self, ctk_img):
        """Update UI in the main thread."""
        self.image_label.configure(image=ctk_img)
        self.image_label.image = ctk_img  # Keep a reference to prevent garbage collection


def main():
    rclpy.init()
    
    # Initialize CustomTkinter UI
    root = ctk.CTk()
    root.geometry("800x600")
    image_label = ctk.CTkLabel(root, text="Video Feed")
    image_label.pack()

    # Initialize ROS2 node
    video_subscriber = VideoSubscriber(image_label, 800, root)

    # Run ROS2 node in a separate thread to keep Tkinter responsive
    thread = threading.Thread(target=rclpy.spin, args=(video_subscriber,), daemon=True)
    thread.start()

    root.mainloop()
    
    video_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
