#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import Altitude
import math
import time
from rclpy.qos import qos_profile_system_default  

class SensorSubscriber(Node):
    def __init__(self, app, x_label, y_label, z_label, battery_percentage_label, 
                 battery_health_label, battery_low_circle, battery_low_circle_id, 
                 horizontal_circle, horizontal_circle_id, vertical_label, 
                 horizontal_label, height_label):
        super().__init__('telemetry_subscriber')
        
        # Create different QoS profiles for different topics
        # MAVROS IMU typically uses sensor data QoS
        imu_qos = qos_profile_sensor_data
        
        # MAVROS battery and other critical topics often use RELIABLE
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Best effort for high-frequency data
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
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
        self.vv = 0.0  # Vertical velocity
        self.hv = 0.0  # Horizontal velocity
        self.altitude = 0.0  # Altitude in meters
        self.battery_percentage = 100.0
        
        # Create subscribers with appropriate QoS profiles
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            imu_qos
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            imu_qos
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity',
            self.velocity_callback,
            imu_qos
        )
        
        self.altitude_sub = self.create_subscription(
            Altitude,
            '/mavros/altitude',
            self.altitude_callback,
            imu_qos
        )
        
        # Track last message time
        self.last_message_time = self.get_clock().now()
        self.publisher_active = True
        self.timer = self.create_timer(5.0, self.check_publisher_status)
    
    def imu_callback(self, msg):
        """Process IMU data (orientation and angular velocity)"""
        self.last_message_time = self.get_clock().now()
        self.publisher_active = True
        
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

        # UI update (same as before)
        if self.x_label:
            self.x_label.configure(text=f"Roll : {self.x:.2f} deg")
            self.y_label.configure(text=f"Pitch : {self.y:.2f} deg")
            self.z_label.configure(text=f"Yaw : {self.z:.2f} deg")
            self.horizontal_circle.itemconfig(self.horizontal_circle_id, fill="#FFFFFF")
            self.app.update_idletasks()
        
    def battery_callback(self, msg):
        """Process battery status"""
        self.last_message_time = self.get_clock().now()
        self.publisher_active = True
        
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
            self.app.update_idletasks()
    
    def velocity_callback(self, msg):
        """Process velocity data"""
        self.last_message_time = self.get_clock().now()
        self.publisher_active = True
        
        self.vv = abs(msg.twist.linear.z)
        self.hv = math.sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2)
        
        if self.vertical_label and self.horizontal_label:
            self.vertical_label.configure(text=f"Vertical : {self.vv:.2f} m/s")
            self.horizontal_label.configure(text=f"Horizontal : {self.hv:.2f} m/s")
            self.app.update_idletasks()
    
    def altitude_callback(self, msg):
        """Process altitude data"""
        self.last_message_time = self.get_clock().now()
        self.publisher_active = True
        
        self.altitude = msg.local * 100  # Convert to cm
        
        if self.height_label:
            self.height_label.configure(text=f"Height : {self.altitude:.2f} cm")
            self.app.update_idletasks()
    
    def check_publisher_status(self):
        """Check if publisher is down"""
        now = self.get_clock().now()
        time_diff = (now - self.last_message_time).nanoseconds / 1e9

        if time_diff > 5.0 and self.publisher_active:
            self.get_logger().warning("Publisher is DOWN! No messages received.")
            self.publisher_active = False
            self.handle_publisher_down()
        elif time_diff <= 5.0 and not self.publisher_active:
            self.publisher_active = True
            self.get_logger().info("Publisher is back ONLINE")

    def handle_publisher_down(self):
        """Update UI when publisher is down"""
        if (self.battery_health_label and self.battery_percentage_label and 
            self.x_label and self.y_label and self.z_label and self.height_label):
            self.battery_health_label.configure(text="No Data", text_color="#FFA500")
            self.battery_percentage_label.configure(text="00 %", text_color="#FFA500")
            self.x_label.configure(text="Roll : 00 deg")
            self.y_label.configure(text="Pitch : 00 deg")
            self.z_label.configure(text="Yaw : 00 deg")
            self.battery_low_circle.itemconfig(self.oval_id, fill="#FFA500")
            self.horizontal_circle.itemconfig(self.horizontal_circle_id, fill="#FF0000")
            self.height_label.configure(text="Height : 00 cm")
            self.app.update_idletasks()


def main(args=None):
    rclpy.init(args=args)
    
    # Initialize with None for all UI elements if running standalone
    node = SensorSubscriber(None, None, None, None, None, None, None, None, None, None, None, None, None)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()