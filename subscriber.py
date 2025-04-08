import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class SensorSubscriber(Node):
    def __init__(self, app, x_label, y_label, z_label, battery_percentage_label, battery_health_label, battery_low_circle, battery_low_circle_id, horizontal_circle, horizontal_circle_id, vertical_label, horizontal_label, height_label):
        super().__init__('sensor_subscriber')

        self.subscription = self.create_subscription(
            String,
            'anav_data',  # ✅ Topic name
            self.listener_callback,
            10
        )

        # Keep track of last received message time
        self.last_message_time = self.get_clock().now()
        self.publisher_active = True  # Flag to track publisher status

        # Timer to check if publisher is down (every 5 seconds)
        self.timer = self.create_timer(5.0, self.check_publisher_status)

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
    def listener_callback(self, msg):
        """Callback function to process received sensor data"""
        try:
            data = json.loads(msg.data)  # ✅ Convert JSON string to dictionary

            # ✅ Update last received time
            self.last_message_time = self.get_clock().now()
            self.publisher_active = True  # Publisher is active

            # Extract angular velocity
            self.x = data.get("imu_data", {}).get(
                "attitude_data", {}).get("roll_deg", 0.0)
            self.y = data.get("imu_data", {}).get(
                "attitude_data", {}).get("pitch_deg", 0.0)
            self.z = data.get("imu_data", {}).get(
                "attitude_data", {}).get("yaw_deg", 0.0)

            # Update UI labels
            self.x_label.configure(text=f"Roll : {self.x:.2f} deg")
            self.y_label.configure(text=f"Pitch : {self.y:.2f} deg")
            self.z_label.configure(text=f"Yaw : {self.z:.2f} deg")

            # Extract battery data
            self.battery_percentage = data.get(
                "battery_data", {}).get("remaining_percent", 100.0)

            self.vv = data.get("imu_data", {}).get(
                "velocity_data", {}).get("vertical_speed", 0.0)

            self.hv = data.get("imu_data", {}).get(
                "velocity_data", {}).get("horizontal_speed", 0.0)
            
            #Extract lidar data
            self.lidar_data = data.get("lidar_data", {}).get("height_cm", 0.0)

            print(data)

            self.vertical_label.configure(text=f"Vertical : {self.vv:.2f} m/s")
            self.horizontal_label.configure(
                text=f"Horizontal : {self.hv:.2f} m/s")
            
            self.height_label.configure(
                text=f"Height : {self.lidar_data:.2f} cm")
            # Update battery health label

            # Update battery UI
            if self.battery_percentage > 50:
                self.battery_percentage_label.configure(
                    text=f"{self.battery_percentage} %", text_color="#00FF00")
                self.battery_health_label.configure(text="Normal", text_color="#00FF00")
                self.battery_low_circle.itemconfig(
                    self.oval_id, fill="#00FF00")
            else:
                self.battery_percentage_label.configure(
                    text=f"{self.battery_percentage} %", text_color="#FF0000")
                self.battery_health_label.configure(
                    text="Danger", text_color="#FF0000")
                self.battery_low_circle.itemconfig(
                    self.oval_id, fill="#FF0000")
                self.horizontal_circle.itemconfig(
                    self.horizontal_circle_id, fill="#FFFFFF")
            self.app.update_idletasks()

        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse JSON data! Invalid format received.")
        except Exception as e:
            self.get_logger().error(f"Unexpected Error: {e}")

    def check_publisher_status(self):
        """Checks if publisher is down based on last received message time"""
        now = self.get_clock().now()
        time_diff = (now - self.last_message_time).nanoseconds / \
            1e9  # Convert to seconds

        if time_diff > 5.0:  # If no message for 5+ seconds
            if self.publisher_active:
                self.get_logger().warning("Publisher is DOWN! No messages received.")
                self.publisher_active = False  # Mark publisher as down
                self.handle_publisher_down()

    def handle_publisher_down(self):
        """Handles UI updates when publisher is down"""
        self.battery_health_label.configure(
            text="No Data", text_color="#FFA500")
        self.battery_percentage_label.configure(
            text="00 %", text_color="#FFA500")
        self.x_label.configure(text="Roll : 00 deg")
        self.y_label.configure(text="Pitch : 00 deg")
        self.z_label.configure(text="Yaw : 00 deg")
        self.battery_low_circle.itemconfig(self.oval_id, fill="#FFA500")
        self.horizontal_circle.itemconfig(
            self.horizontal_circle_id, fill="#FF0000")
        self.app.update_idletasks()


def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
