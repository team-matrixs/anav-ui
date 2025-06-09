#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import Altitude
import math
import time

class SensorSubscriber:
    def __init__(self, app, x_label, y_label, z_label, battery_percentage_label, 
                 battery_health_label, battery_low_circle, battery_low_circle_id, 
                 horizontal_circle, horizontal_circle_id, vertical_label, 
                 horizontal_label, height_label):
        
        # Store UI components
        self.app = app
        self.x_label = x_label
        self.y_label = y_label
        self.z_label = z_label
        self.battery_percentage_label = battery_percentage_label
        self.battery_health_label = battery_health_label
        self.battery_low_circle = battery_low_circle
        self.oval_id = battery_low_circle_id
        self.horizontal_circle = horizontal_circle
        self.horizontal_circle_id = horizontal_circle_id
        self.vertical_label = vertical_label
        self.horizontal_label = horizontal_label
        self.height_label = height_label
        
        # Initialize data variables
        self.x = 0.0  # Roll
        self.y = 0.0  # Pitch
        self.z = 0.0  # Yaw
        self.vv = 0.0  # Vertical velocity (m/s, positive up)
        self.hv = 0.0  # Horizontal velocity (m/s)
        self.hv_north = 0.0  # North component of horizontal velocity
        self.hv_east = 0.0  # East component of horizontal velocity
        self.altitude = 0.0  # Altitude in meters
        self.height_above_home = 0.0  # Height above home position
        self.battery_percentage = 100.0
        
        # Connection management
        self.last_message_time = rospy.Time.now()
        self.connection_active = True
        self.subscribers = {}
        self.connection_timer = None
        self.reconnect_timer = None
        
        # Initialize subscribers
        self.initialize_subscribers()
        
        # Start connection monitor
        self.start_connection_monitor()
    
    def initialize_subscribers(self):
        """Initialize or reinitialize all subscribers"""
        # Unregister any existing subscribers
        for sub in self.subscribers.values():
            sub.unregister()
        self.subscribers.clear()
        
        try:
            # Create new subscribers
            self.subscribers['imu'] = rospy.Subscriber(
                '/mavros/imu/data',
                Imu,
                self.imu_callback,
                queue_size=10
            )
            
            self.subscribers['battery'] = rospy.Subscriber(
                '/mavros/battery',
                BatteryState,
                self.battery_callback,
                queue_size=10
            )
            
            # Velocity subscribers
            self.subscribers['velocity_local'] = rospy.Subscriber(
                '/mavros/local_position/velocity_local',
                TwistStamped,
                self.velocity_local_callback,
                queue_size=10
            )
            
            # Altitude subscribers
            self.subscribers['altitude'] = rospy.Subscriber(
                '/mavros/altitude',
                Altitude,
                self.altitude_callback,
                queue_size=10
            )
            
            self.subscribers['local_position'] = rospy.Subscriber(
                '/mavros/local_position/pose',
                PoseStamped,
                self.local_position_callback,
                queue_size=10
            )
            
            rospy.loginfo("Subscribers initialized successfully")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize subscribers: {str(e)}")
            return False
    
    def velocity_local_callback(self, msg):
        """Process local velocity data (includes vertical and horizontal components)"""
        self.update_last_message_time()
        
        # Vertical velocity (positive up)
        self.vv = msg.twist.linear.z
        
        # Horizontal velocity components
        self.hv_north = msg.twist.linear.x  # North component
        self.hv_east = msg.twist.linear.y   # East component
        
        # Calculate total horizontal speed
        self.hv = math.sqrt(self.hv_north**2 + self.hv_east**2)
        
        # UI update
        if self.vertical_label and self.horizontal_label:
            self.vertical_label.configure(text=f"Vertical: {self.vv:.2f} m/s")
            self.horizontal_label.configure(text=f"Horizontal: {self.hv:.2f} m/s")
            self.safe_update_ui()
    
    def local_position_callback(self, msg):
        """Process local position data to get height above home"""
        self.update_last_message_time()
        
        # Height above home position (positive up)
        self.height_above_home = msg.pose.position.z
        
        # UI update
        if self.height_label:
            self.height_label.configure(text=f"Height: {self.height_above_home:.2f} m")
            self.safe_update_ui()
    
    def start_connection_monitor(self):
        """Start or restart the connection monitoring timer"""
        if self.connection_timer is not None:
            self.connection_timer.shutdown()
        self.connection_timer = rospy.Timer(rospy.Duration(1.0), self.check_connection_status)
    
    def start_reconnect_attempts(self):
        """Start periodic reconnect attempts"""
        if self.reconnect_timer is not None:
            self.reconnect_timer.shutdown()
        self.reconnect_timer = rospy.Timer(rospy.Duration(5.0), self.attempt_reconnect)
    
    def stop_reconnect_attempts(self):
        """Stop periodic reconnect attempts"""
        if self.reconnect_timer is not None:
            self.reconnect_timer.shutdown()
            self.reconnect_timer = None
    
    def imu_callback(self, msg):
        """Process IMU data (orientation and angular velocity)"""
        self.update_last_message_time()
        
        # Convert quaternion to Euler angles (simplified)
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.x = math.atan2(sinr_cosp, cosr_cosp) * (180 / math.pi)

        sinp = 2 * (w * y - z * x)
        pitch_rad = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        self.y = -pitch_rad * (180 / math.pi) 
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.z = math.atan2(siny_cosp, cosy_cosp) * (180 / math.pi)

        # UI update
        if self.x_label:
            self.x_label.configure(text=f"Roll : {self.x:.2f} deg")
            self.y_label.configure(text=f"Pitch : {self.y:.2f} deg")
            self.z_label.configure(text=f"Yaw : {self.z:.2f} deg")
            self.horizontal_circle.itemconfig(self.horizontal_circle_id, fill="#FFFFFF")
            self.safe_update_ui()
        
    def battery_callback(self, msg):
        """Process battery status"""
        self.update_last_message_time()
        
        self.battery_percentage = msg.percentage * 100
        
        if self.battery_percentage_label and self.battery_health_label:
            if self.battery_percentage > 30:
                self.battery_percentage_label.configure(
                    text=f"{self.battery_percentage:.0f} %", text_color="#00FF00")
                self.battery_health_label.configure(text="Normal", text_color="#00FF00")
                self.battery_low_circle.itemconfig(self.oval_id, fill="#00FF00")
            else:
                self.battery_percentage_label.configure(
                    text=f"{self.battery_percentage:.0f} %", text_color="#FF0000")
                self.battery_health_label.configure(text="Low", text_color="#FF0000")
                self.battery_low_circle.itemconfig(self.oval_id, fill="#FF0000")
                self.horizontal_circle.itemconfig(self.horizontal_circle_id, fill="#FFFFFF")
            self.safe_update_ui()
    
    def altitude_callback(self, msg):
        """Process altitude data"""
        self.update_last_message_time()
        
        self.altitude = msg.local  # Altitude in meters
        
        # UI update (keeping this as backup, though height_above_home might be more relevant)
        if self.height_label and not hasattr(self, 'height_above_home'):
            self.height_label.configure(text=f"Altitude: {self.altitude:.2f} m")
            self.safe_update_ui()
    
    def update_last_message_time(self):
        """Update the last message time and handle reconnection if needed"""
        self.last_message_time = rospy.Time.now()
        if not self.connection_active:
            self.connection_active = True
            self.stop_reconnect_attempts()
            rospy.loginfo("Connection restored - publisher is back ONLINE")
            self.handle_connection_restored()
    
    def check_connection_status(self, event=None):
        """Check if connection is lost and handle accordingly"""
        now = rospy.Time.now()
        time_diff = (now - self.last_message_time).to_sec()

        if time_diff > 5.0:
            if self.connection_active:
                rospy.logwarn(f"Connection lost! No messages received for {time_diff:.1f} seconds")
                self.connection_active = False
                self.handle_connection_lost()
                self.start_reconnect_attempts()
            else:
                # Connection remains lost - update UI to show we're still disconnected
                self.handle_connection_lost()
        else:
            if not self.connection_active:
                self.connection_active = True
                self.stop_reconnect_attempts()
                rospy.loginfo("Connection restored - publisher is back ONLINE")
                self.handle_connection_restored()
    
    def attempt_reconnect(self, event=None):
        """Attempt to reconnect to ROS topics"""
        if not self.connection_active:
            rospy.loginfo("Attempting to reconnect...")
            success = self.initialize_subscribers()
            if success:
                # Give it a moment to receive messages
                rospy.sleep(1.0)
            else:
                rospy.logwarn("Reconnect attempt failed")
    
    def handle_connection_lost(self):
        """Handle UI updates when connection is lost"""
        try:
            if (self.battery_health_label and self.battery_percentage_label and 
                self.x_label and self.y_label and self.z_label and self.height_label):
                self.battery_health_label.configure(text="No Data", text_color="#FFA500")
                self.battery_percentage_label.configure(text="00 %", text_color="#FFA500")
                self.x_label.configure(text="Roll : 00 deg")
                self.y_label.configure(text="Pitch : 00 deg")
                self.z_label.configure(text="Yaw : 00 deg")
                self.battery_low_circle.itemconfig(self.oval_id, fill="#FFA500")
                self.horizontal_circle.itemconfig(self.horizontal_circle_id, fill="#FF0000")
                self.height_label.configure(text="Height : 00 m")
                self.vertical_label.configure(text="Vertical: 0.00 m/s")
                self.horizontal_label.configure(text="Horizontal: 0.00 m/s")
                self.safe_update_ui()
        except Exception as e:
            rospy.logerr(f"Error in handle_connection_lost: {str(e)}")
    
    def handle_connection_restored(self):
        """Handle UI updates when connection is restored"""
        # No special handling needed as callbacks will update UI
        pass
    
    def safe_update_ui(self):
        """Safely update the UI, handling cases where app might be None"""
        if self.app is not None:
            try:
                self.app.update_idletasks()
            except Exception as e:
                rospy.logwarn(f"UI update failed: {str(e)}")