import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge


class CameraSubscriber:
    """ROS Node for subscribing to camera feed"""
    
    def __init__(self, update_callback):
        # rospy.init_node('camera_subscriber', anonymous=True)
        self.update_callback = update_callback
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        self.subscription = rospy.Subscriber(
            '/camera/camera/color/image_raw',  # Change this to your actual camera topic
            ROSImage,
            self.image_callback,
            queue_size=10)
    
    def image_callback(self, msg):
        """Callback for when a new image message is received"""
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Call the update callback with the new image
            self.update_callback(cv_image)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")