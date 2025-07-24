#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu, BatteryState, Range
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import Altitude
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time

class SensorSubscriber(Node):
    def __init__(self, ui=None):
        super().__init__('sensor_subscriber')
        
        # Store reference to the UI
        self.ui = ui
        
        # Initialize sensor data variables
        self.x = self.y = self.z = 0.0
        self.vv = self.hv = self.hv_north = self.hv_east = 0.0
        self.altitude = self.height_above_home = 0.0
        self.rangefinder_height = 0.0
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_status = "Unknown"
        self.safe_points = ["1 : X:00.00 Y:00.00", "2 : X:00.00 Y:00.00", "3 : X:00.00 Y:00.00"]  # Default safe points
        self.rangefinder_available = False
        self.last_rangefinder_time = 0.0 
        
        self.last_message_time = self.get_clock().now()
        self.connection_active = True

        # Initialize subscribers and timers
        self.init_subscribers()
        self.connection_timer = self.create_timer(1.0, self.check_connection_status)
        # self.rangefinder_check_timer = self.create_timer(0.5, self.check_rangefinder_status)

    def init_subscribers(self):
        self.mavros_qos = QoSProfile(depth=10)
        self.mavros_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Create all subscribers
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, self.mavros_qos)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', 
                                self.velocity_local_callback, self.mavros_qos)
        self.create_subscription(BatteryState, '/mavros/battery', 
                                self.battery_callback, self.mavros_qos)
        self.create_subscription(Range, '/mavros/rangefinder/rangefinder', 
                                self.rangefinder_callback, self.mavros_qos)
        self.create_subscription(String, '/text_topic', self.text_topic_callback, self.mavros_qos)

        self.get_logger().info("All subscribers initialized.")

    def battery_callback(self, msg):
        """Callback for battery data"""
        self.update_last_message_time()
        self.battery_percentage = msg.percentage * 100  # Convert from 0-1 to percentage
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        
        # Map power supply status to human-readable string
        status_map = {
            0: "Unknown",
            1: "Charging",
            2: "Discharging",
            3: "Not Charging",
            4: "Full"
        }
        self.battery_status = status_map.get(msg.power_supply_status, "Unknown")
        
        self.update_battery_display()

    def text_topic_callback(self, msg):
        """Callback for text topic containing safe points"""
        self.update_last_message_time()
        try:
            # Split the message into lines and update safe points
            lines = msg.data.split('\n')
            for i, line in enumerate(lines[:3]):  # Only take first 3 lines if available
                self.safe_points[i] = line.strip()
            
            # Update UI labels if they exist
            if self.ui:
                if hasattr(self.ui, 'label_13') and len(self.safe_points) > 0:  # Safe point 1
                    self.ui.label_13.setText(self.safe_points[0])
                if hasattr(self.ui, 'label_15') and len(self.safe_points) > 1:  # Safe point 2
                    self.ui.label_15.setText(self.safe_points[1])
                if hasattr(self.ui, 'label_16') and len(self.safe_points) > 2:  # Safe point 3
                    self.ui.label_16.setText(self.safe_points[2])
        except Exception as e:
            self.get_logger().error(f"Error processing text topic: {str(e)}")

    def check_rangefinder_status(self):
        """Check if we're receiving rangefinder data"""
        current_time = time.time()
        if current_time - self.last_rangefinder_time > 5.0 and self.rangefinder_available:
            self.rangefinder_available = False
            self.get_logger().warn("Rangefinder data not received recently")
            self.update_height_display()
        elif current_time - self.last_rangefinder_time <= 5.0 and not self.rangefinder_available:
            self.rangefinder_available = True
            self.get_logger().info("Rangefinder data available")

    def rangefinder_callback(self, msg):
        """Callback for rangefinder data"""
        self.update_last_message_time()
        self.rangefinder_height = msg.range
        self.last_rangefinder_time = time.time()
        self.rangefinder_available = True
        self.update_height_display()

    def update_height_display(self):
        """Update height display with rangefinder data if available"""
        if self.ui and hasattr(self.ui, 'label_19'):  # Altitude label
            if self.rangefinder_available:
                text = f"{self.rangefinder_height:.1f}"
            else:
                text = "N/A"
            self.ui.label_19.setText(text)

            
    def update_battery_display(self):
        """Update battery display based on current percentage"""
        try:
            # Determine battery health and color
            if self.battery_percentage < 10:  # Critical below 10%
                health_color = "#FF0000"  # Red
                battery_health = "Critical"
            elif self.battery_percentage < 30:  # Low below 30%
                health_color = "#FFA500"  # Orange
                battery_health = "Low"
            else:
                health_color = "#00FF3B"  # Green
                battery_health = "Normal"
                
            # Update UI elements if they exist
            if self.ui:
                if hasattr(self.ui, 'label_21'):  # Battery percentage
                    self.ui.label_21.setText(f"{abs(self.battery_percentage):.1f}%")
                
                # Update status indicator dots
                if hasattr(self.ui, 'widget_12'):
                    self.ui.widget_12.setStyleSheet(f"border-radius: 12px; border: none; background-color: {health_color};")
            
        except Exception as e:
            self.get_logger().error(f"Error updating battery display: {str(e)}")

    def imu_callback(self, msg):
        """Callback for IMU data (orientation)"""
        self.update_last_message_time()
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z

        # Convert quaternion to Euler angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.x = math.atan2(sinr_cosp, cosr_cosp) * 180.0 / math.pi

        sinp = 2 * (w * y - z * x)
        self.y = -math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else -math.asin(sinp) * 180.0 / math.pi

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.z = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi

        # Update orientation labels if they exist
        if self.ui:
            if hasattr(self.ui, 'label_7'):  # X orientation
                self.ui.label_7.setText(f"X : {self.x:.2f}")
            if hasattr(self.ui, 'label_8'):  # Y orientation
                self.ui.label_8.setText(f"Y : {self.y:.2f}")
            if hasattr(self.ui, 'label_9'):  # Z orientation
                self.ui.label_9.setText(f"Z : {self.z:.2f}")

    def velocity_local_callback(self, msg):
        """Callback for velocity data"""
        self.update_last_message_time()
        self.vv = msg.twist.linear.z
        self.hv_north = msg.twist.linear.x
        self.hv_east = msg.twist.linear.y
        self.hv = math.sqrt(self.hv_north**2 + self.hv_east**2)

        # Update velocity labels if they exist
        if self.ui:
            if hasattr(self.ui, 'label_10'):  # X velocity
                self.ui.label_10.setText(f"X : {self.hv_north:.2f}")
            if hasattr(self.ui, 'label_11'):  # Y velocity
                self.ui.label_11.setText(f"Y : {self.hv_east:.2f}")
            if hasattr(self.ui, 'label_12'):  # Z velocity
                self.ui.label_12.setText(f"Z : {self.vv:.2f}")
            # self.update_height_display()

    def local_position_callback(self, msg):
        """Callback for local position data"""
        self.update_last_message_time()
        self.height_above_home = msg.pose.position.z
        self.update_height_display()

    def check_connection_status(self):
        """Check if we're still receiving messages"""
        now = self.get_clock().now()
        time_diff = (now - self.last_message_time).nanoseconds / 1e9
        
        if time_diff > 5.0 and self.connection_active:
            self.connection_active = False
            self.get_logger().warn(f"Connection lost! No messages received for {time_diff:.1f} seconds")
            self.handle_connection_lost()
        elif time_diff <= 5.0 and not self.connection_active:
            self.connection_active = True
            self.get_logger().info("Connection restored")
            self.handle_connection_restored()

    def update_last_message_time(self):
        """Update the last message received time"""
        self.last_message_time = self.get_clock().now()

    def handle_connection_lost(self):
        """Handle connection loss by resetting displayed values"""
        try:
            if self.ui:
                # Reset orientation displays
                if hasattr(self.ui, 'label_7'):
                    self.ui.label_7.setText("X : 00.00")
                if hasattr(self.ui, 'label_8'):
                    self.ui.label_8.setText("Y : 00.00")
                if hasattr(self.ui, 'label_9'):
                    self.ui.label_9.setText("Z : 00.00")
                
                # Reset velocity displays
                if hasattr(self.ui, 'label_10'):
                    self.ui.label_10.setText("X : 00.00")
                if hasattr(self.ui, 'label_11'):
                    self.ui.label_11.setText("Y : 00.00")
                if hasattr(self.ui, 'label_12'):
                    self.ui.label_12.setText("Z : 00.00")
                
                # Update connection status indicator
                if hasattr(self.ui, 'widget_13'):
                    self.ui.widget_13.setStyleSheet("border-radius: 12px; border: none; background-color: #FF0000;")
                
                # Reset safe point displays
                if hasattr(self.ui, 'label_13'):
                    self.ui.label_13.setText("1 : X:00.00 Y:00.00")
                if hasattr(self.ui, 'label_15'):
                    self.ui.label_15.setText("2 : X:00.00 Y:00.00")
                if hasattr(self.ui, 'label_16'):
                    self.ui.label_16.setText("3 : X:00.00 Y:00.00")
                
        except Exception as e:
            self.get_logger().error(f"UI error on connection lost: {str(e)}")

    def handle_connection_restored(self):
        """Handle connection restoration"""
        if self.ui:
            # Update connection status indicator
            if hasattr(self.ui, 'widget_13'):
                self.ui.widget_13.setStyleSheet("border-radius: 12px; border: none; background-color: #A4A4A5;")
        

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()