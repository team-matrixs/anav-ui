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
    def __init__(self, app=None, x_label=None, y_label=None, z_label=None,
                 battery_percentage_label=None, battery_health_label=None,
                 battery_low_circle=None, battery_low_circle_id=None,
                 horizontal_circle=None, horizontal_circle_id=None,
                 vertical_label=None, horizontal_label=None, height_label=None,
                 safe_label_x=None, safe_label_y=None, safe_label_z=None):
        super().__init__('sensor_subscriber')

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
        self.safe_label_x = safe_label_x
        self.safe_label_y = safe_label_y
        self.safe_label_z = safe_label_z

        self.x = self.y = self.z = 0.0
        self.vv = self.hv = self.hv_north = self.hv_east = 0.0
        self.altitude = self.height_above_home = 0.0
        self.rangefinder_height = 0.0  # New variable for rangefinder height
        self.battery_percentage = 83
        self.last_battery_update = time.time()
        self.safe_x = self.safe_y = self.safe_z = ""
        self.rangefinder_available = False  # Track rangefinder status
        self.last_rangefinder_time = 0.0 

        self.last_message_time = self.get_clock().now()
        self.connection_active = True

        self.init_subscribers()
        self.update_battery()
        self.connection_timer = self.create_timer(1.0, self.check_connection_status)
        self.battery_timer = self.create_timer(60.0, self.update_battery)  # Update battery every minute
        self.rangefinder_check_timer = self.create_timer(5.0, self.check_rangefinder_status)

    def init_subscribers(self):
        self.mavros_qos = QoSProfile(depth=10)
        self.mavros_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, self.mavros_qos)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.velocity_local_callback, self.mavros_qos)
        self.create_subscription(Altitude, '/mavros/altitude', self.altitude_callback, self.mavros_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_position_callback, self.mavros_qos)
        # Add rangefinder subscriber
        self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self.rangefinder_callback, self.mavros_qos)
        self.create_subscription(String, '/x_coordinate', self.safe_point_callback_x, self.mavros_qos)
        self.create_subscription(String, '/y_coordinate', self.safe_point_callback_z, self.mavros_qos)
        self.create_subscription(String, '/z_coordinate', self.safe_point_callback_y, self.mavros_qos)

        self.get_logger().info("All subscribers initialized.")

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
        if self.height_label:
            display_text = f"Height: {self.rangefinder_height:.2f} m"
            self.height_label.configure(text=display_text)
            self.safe_update_ui()

    def update_battery(self):
        """Decrease battery percentage by 1% each minute"""
        if self.battery_percentage > 0:
            self.battery_percentage -= 2.0
            self.update_battery_display()

    def update_battery_display(self):
        """Update battery display based on current percentage"""
        try:
            # Determine battery health
            if self.battery_percentage < 10:  # Critical below 10%
                battery_health = "Critical"
                health_color = "#FF0000"  # Red
            elif self.battery_percentage < 30:  # Low below 30%
                battery_health = "Low"
                health_color = "#FFA500"  # Orange
            else:
                battery_health = "Normal"
                health_color = "#00FF3B"  # Green
                
            # Update GUI on main thread
            if self.app:
                self.app.after(0, lambda: [
                    self.battery_percentage_label.configure(text=f"{self.battery_percentage:.0f}%", text_color=health_color),
                    self.battery_health_label.configure(text=battery_health, text_color=health_color),
                    self.battery_low_circle.itemconfig(self.oval_id, fill=health_color)
                ])
            
        except Exception as e:
            self.get_logger().error(f"Error updating battery display: {str(e)}")

    def imu_callback(self, msg):
        self.update_last_message_time()
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.x = math.atan2(sinr_cosp, cosr_cosp) * 180.0 / math.pi

        sinp = 2 * (w * y - z * x)
        self.y = -math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else -math.asin(sinp) * 180.0 / math.pi

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.z = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi

        if self.x_label:
            self.x_label.configure(text=f"Roll : {self.x:.2f} deg")
            self.y_label.configure(text=f"Pitch : {self.y:.2f} deg")
            self.z_label.configure(text=f"Yaw : {self.z:.2f} deg")
            self.horizontal_circle.itemconfig(self.horizontal_circle_id, fill="#FFFFFF")
            self.safe_update_ui()

    def velocity_local_callback(self, msg):
        self.update_last_message_time()
        self.vv = msg.twist.linear.z
        self.hv_north = msg.twist.linear.x
        self.hv_east = msg.twist.linear.y
        self.hv = math.sqrt(self.hv_north**2 + self.hv_east**2)

        if self.vertical_label:
            self.vertical_label.configure(text=f"Vertical: {self.vv:.2f} m/s")
            self.horizontal_label.configure(text=f"Horizontal: {self.hv:.2f} m/s")
            self.safe_update_ui()

    def local_position_callback(self, msg):
        self.update_last_message_time()
        self.height_above_home = msg.pose.position.z
        self.update_height_display()

    def altitude_callback(self, msg):
        self.update_last_message_time()
        self.altitude = msg.local
        if self.height_label:
            if self.rangefinder_available:
                display_text = f"Altitude: {self.altitude:.2f} m (Rangefinder: {self.rangefinder_height:.2f} m)"
            else:
                display_text = f"Altitude: {self.altitude:.2f} m (Rangefinder: N/A)"
            self.height_label.configure(text=display_text)
            self.safe_update_ui()

    def safe_point_callback_x(self, msg):
        self.update_last_message_time()
        self.safe_x = msg.data
        if self.safe_label_x:
            self.safe_label_x.configure(text=f"1: {self.safe_x}")

    def safe_point_callback_y(self, msg):
        self.update_last_message_time()
        self.safe_z = msg.data
        if self.safe_label_z:
            self.safe_label_z.configure(text=f"3: {self.safe_z}")

    def safe_point_callback_z(self, msg):
        self.update_last_message_time()
        self.safe_y = msg.data
        if self.safe_label_y:
            self.safe_label_y.configure(text=f"2: {self.safe_y}")

    def check_connection_status(self):
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
        self.last_message_time = self.get_clock().now()

    def handle_connection_lost(self):
        try:
            if self.battery_health_label:
                self.x_label.configure(text="Roll : 00 deg")
                self.y_label.configure(text="Pitch : 00 deg")
                self.z_label.configure(text="Yaw : 00 deg")
                self.horizontal_circle.itemconfig(self.horizontal_circle_id, fill="#FF0000")
                self.height_label.configure(text="Height : 00 m")
                self.vertical_label.configure(text="Vertical: 0.00 m/s")
                self.horizontal_label.configure(text="Horizontal: 0.00 m/s")
                self.safe_update_ui()
        except Exception as e:
            self.get_logger().error(f"UI error on connection lost: {str(e)}")

    def handle_connection_restored(self):
        pass

    def safe_update_ui(self):
        if self.app:
            try:
                self.app.update_idletasks()
            except Exception as e:
                self.get_logger().warn(f"UI update failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()