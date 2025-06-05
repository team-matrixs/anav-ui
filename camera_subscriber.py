from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge


class CameraSubscriber(Node):
    """ROS2 Node for subscribing to camera feed"""
    
    def __init__(self, update_callback):
        super().__init__('camera_subscriber')
        self.update_callback = update_callback
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            ROSImage,
            '/camera/camera/color/image_raw',  # Change this to your actual camera topic
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning
    
    def image_callback(self, msg):
        """Callback for when a new image message is received"""
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Call the update callback with the new image
            self.update_callback(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

