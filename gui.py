import customtkinter as ctk
from PIL import Image
import os
import tkinter as tk
from subscriber import SensorSubscriber
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import asyncio
import threading
from camera_sub import VideoSubscriber
from rclpy.executors import MultiThreadedExecutor


# class IMU_DATA:
#     def __init__(self):
#         self.x = 0.0
#         self.y = 0.0
#         self.z = 0.0


class FlightDataApp():

    def __init__(self):
        self.app = ctk.CTk()
        self.app.title("Flight Data - (Team Matrix)")

        # Get screen dimensions
        self.screen_width = self.app.winfo_screenwidth()
        self.screen_height = self.app.winfo_screenheight()

        # Set window size
        self.app.geometry(f"{self.screen_width}x{self.screen_height}")

        # Configure grid for responsiveness
        self.app.grid_columnconfigure(0, weight=1)
        self.app.grid_rowconfigure(1, weight=0)

        # Create UI components
        self.create_nav_bar()
        self.create_telemetry_section()
        self.create_camera_section()

        # Initialize ROS 2
        rclpy.init()

        # Create ROS 2 nodes
        self.imu_node = SensorSubscriber(
            self.app, self.x_label, self.y_label, self.z_label, self.status_label2, self.status_label4, self.vertical_circle, self.battery_low_circle_id, self.horizontal_circle, self.horizontal_circle_id, self.vertical_label, self.horizontal_label)
        # self.camera_node = VideoSubscriber(self.image_label, self.screen_width)

        # Use MultiThreadedExecutor for both nodes
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.imu_node)
        # self.executor.add_node(self.camera_node)

        # Start ROS 2 in a separate thread
        self.ros_thread = threading.Thread(
            target=self.run_ros_executor)
        self.ros_thread.start()

        # Handle app close event
        self.app.protocol("WM_DELETE_WINDOW", self.on_close)

    def run_ros_executor(self):
        """Run the ROS 2 executor to manage multiple nodes safely."""
        try:
            self.executor.spin()
        except Exception as e:
            print(f"ROS Executor Error: {e}")
        finally:
            self.executor.shutdown()

    def on_close(self):
        """Handles application exit gracefully."""
        print("Shutting down...")

        # Stop the executor
        self.executor.shutdown()

        # Destroy nodes
        self.imu_node.destroy_node()
        # self.camera_node.destroy_node()

        # Shutdown ROS
        rclpy.shutdown()

        # Close UI
        self.app.destroy()

    def create_nav_bar(self):
        # Navigation bar
        self.nav_bar = ctk.CTkFrame(self.app, fg_color="#E53935")
        self.nav_bar.grid(row=0, column=0, sticky="ew")
        self.nav_bar.grid_columnconfigure(0, weight=1)
        self.nav_bar.grid_columnconfigure(1, weight=1)

        # Navigation heading
        self.navbar_heading = ctk.CTkLabel(
            self.nav_bar, text="Flight Data", font=("Arial", 30, "bold"))
        self.navbar_heading.grid(row=0, column=0, sticky="w", padx=30, pady=20)

        # Status box
        self.create_status_box()

    def create_status_box(self):
        self.navbar_status_box = ctk.CTkFrame(self.nav_bar, height=90)
        self.navbar_status_box.grid(
            row=0, column=1, sticky="e", padx=20, pady=20)

        self.navbar_status_box_1 = ctk.CTkFrame(
            self.navbar_status_box, fg_color="transparent")
        self.navbar_status_box_1.grid(
            row=0, column=1, sticky="e", padx=20, pady=20)

        # Status labels inside the status box
        self.status_label1 = ctk.CTkLabel(
            self.navbar_status_box_1, text="Battery Status: ", font=("Arial", 21, "bold"))
        self.status_label1.pack(side="left", padx=0, pady=10)

        self.status_label2 = ctk.CTkLabel(self.navbar_status_box_1, text="", font=(
            "Arial", 21, "bold"), text_color="#00FF3B")
        self.status_label2.pack(side="left", padx=20, pady=10)

        self.status_label3 = ctk.CTkLabel(
            self.navbar_status_box_1, text="Battery Health: ", font=("Arial", 21, "bold"))
        self.status_label3.pack(side="left", padx=20, pady=10)

        self.status_label4 = ctk.CTkLabel(self.navbar_status_box_1, text="", font=(
            "Arial", 21, "bold"), text_color="#00FF3B")
        self.status_label4.pack(side="left", padx=0, pady=10)

    def create_telemetry_section(self):
        # Telemetry Heading (Directly below navbar)
        self.telemetry_heading = ctk.CTkLabel(self.app, text="Telemetry Information", font=(
            "Arial", 24, "bold"), fg_color="transparent")
        self.telemetry_heading.grid(
            row=1, column=0, padx=40, pady=50, sticky="w")

        # Telemetry Data Box (Fills remaining space)
        self.telemetry_data_box = ctk.CTkFrame(
            self.app, corner_radius=10, height=500, fg_color="transparent")
        self.telemetry_data_box.grid(
            row=2, column=0, sticky="ew", padx=20, pady=10)
        self.telemetry_data_box.grid_rowconfigure(1, weight=1)
        self.telemetry_data_box.grid_columnconfigure(2, weight=1)

        # Telemetry Data Box (Fixed Width)
        self.telemetry_data_box_column1 = ctk.CTkFrame(self.telemetry_data_box, height=500, width=int(
            self.screen_width * 0.85), fg_color="#E53935", corner_radius=20)
        self.telemetry_data_box_column1.grid(
            row=1, column=0, sticky="nsew", padx=20, pady=20)

        # Prevent the frame from resizing automatically
        self.telemetry_data_box_column1.grid_propagate(False)

        # Set the telemetry box column to a fixed width
        self.telemetry_data_box.grid_columnconfigure(
            0, weight=0)  # No auto expansion

        # Create Columns for Fixed Layout
        self.telemetry_data_box_column1.grid_columnconfigure(0, weight=1)
        self.telemetry_data_box_column1.grid_columnconfigure(1, weight=1)
        self.telemetry_data_box_column1.grid_columnconfigure(2, weight=1)
        self.telemetry_data_box_column1.grid_columnconfigure(3, weight=1)

        # Align items to the center
        self.telemetry_data_box_column1.grid_rowconfigure(
            0, weight=1)  # Center vertically
        self.telemetry_data_box_column1.grid_rowconfigure(1, weight=0)
        self.telemetry_data_box_column1.grid_rowconfigure(2, weight=1)

        # Create telemetry boxes
        self.create_angular_velocity_box()
        self.create_velocity_box()
        self.create_safe_site_box()
        self.create_safe_mode_box()
        self.create_control_mode_box()

    def create_angular_velocity_box(self):
        # Angular velocity box with padding
        self.Angular_velocity = ctk.CTkFrame(
            self.telemetry_data_box_column1, height=700, width=600, fg_color="#2B2828", corner_radius=20)
        self.Angular_velocity.grid(
            row=1, column=0, padx=20, pady=10, sticky="nsew")

        # Add padding inside the frame using an inner container
        self.inner_padding_frame = ctk.CTkFrame(
            self.Angular_velocity, fg_color="transparent")
        self.inner_padding_frame.pack(
            fill="both", expand=True, padx=0, pady=20)

        # Configure grid for centering
        self.inner_padding_frame.grid_columnconfigure(
            (0, 1, 2, 4, 5), weight=1)
        self.inner_padding_frame.grid_rowconfigure((0, 1, 2), weight=1)

        # Add the "Angular Velocity" label at the top center
        self.angular_velocity_label = ctk.CTkLabel(
            self.inner_padding_frame, text="Orientation", font=("Arial", 24, "bold"), text_color="#FFFFFF")
        self.angular_velocity_label.grid(
            row=0, column=2,  padx=5, pady=10, sticky="n")

        # Center the X, Y, Z labels in the middle
        self.x_label = ctk.CTkLabel(self.inner_padding_frame, text="Roll : 00 deg", font=(
            "Arial", 20, "bold"), text_color="#0048FF")
        self.x_label.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

        self.y_label = ctk.CTkLabel(self.inner_padding_frame, text="Pitch : 00 deg", font=(
            "Arial", 20, "bold"), text_color="#19FF00")
        self.y_label.grid(row=1, column=2, padx=10, pady=10, sticky="nsew")

        self.z_label = ctk.CTkLabel(self.inner_padding_frame, text="Yaw : 00 deg", font=(
            "Arial", 20, "bold"), text_color="#FF0000")
        self.z_label.grid(row=1, column=4, padx=10, pady=10, sticky="nsew")

    def create_velocity_box(self):
        # Velocity box
        self.Velocity = ctk.CTkFrame(
            self.telemetry_data_box_column1, height=600, width=400, fg_color="#2B2828", corner_radius=20)
        self.Velocity.grid(row=1, column=1, padx=20, pady=10, sticky="nsew")

        # Add padding inside the frame using an inner container
        self.inner_padding_velocity = ctk.CTkFrame(
            self.Velocity, fg_color="transparent")
        self.inner_padding_velocity.pack(
            fill="both", expand=True, padx=20, pady=20)

        # Configure grid to center elements
        self.inner_padding_velocity.grid_columnconfigure(0, weight=1)
        self.inner_padding_velocity.grid_rowconfigure((0, 1, 2), weight=1)

        # Add the "Velocity" label at the top center
        self.velocity_label = ctk.CTkLabel(self.inner_padding_velocity, text="Velocity", font=(
            "Arial", 24, "bold"), text_color="#FFFFFF")
        self.velocity_label.grid(
            row=0, column=0, padx=10, pady=(10, 20), sticky="n")

        # Create a small frame to group the Vertical and Horizontal labels
        self.velocity_values_frame = ctk.CTkFrame(
            self.inner_padding_velocity, fg_color="transparent")
        self.velocity_values_frame.grid(row=1, column=0, sticky="nsew")

        # Center the Vertical and Horizontal labels inside the frame
        self.vertical_label = ctk.CTkLabel(self.velocity_values_frame, text="Vertical : 00 m/s", font=(
            "Arial", 20, "bold"), text_color="#FF0000")
        self.vertical_label.pack(pady=20)

        self.horizontal_label = ctk.CTkLabel(self.velocity_values_frame, text="Horizontal : 00 m/s", font=(
            "Arial", 20, "bold"), text_color="#11FF00")
        self.horizontal_label.pack(pady=20)

    def create_safe_site_box(self):
        # Safe Site box
        self.Safe_Site = ctk.CTkFrame(
            self.telemetry_data_box_column1, height=600, width=400, fg_color="#2B2828", corner_radius=20)
        self.Safe_Site.grid(row=1, column=2, padx=20, pady=5, sticky="nsew")

        # Add padding inside the frame using an inner container
        self.inner_padding_frame_safe = ctk.CTkFrame(
            self.Safe_Site, fg_color="transparent")
        self.inner_padding_frame_safe.pack(
            fill="both", expand=True, padx=5, pady=20)

        # Configure grid for centering
        self.inner_padding_frame_safe.grid_columnconfigure((0, 1, 2), weight=1)
        self.inner_padding_frame_safe.grid_rowconfigure((0, 1, 2), weight=1)

        # Add the "Safe Site Co-Ordinates" label at the top center
        self.safe_label = ctk.CTkLabel(
            self.inner_padding_frame_safe, text="Safe Site Co-Ordinates", font=("Arial", 24, "bold"), text_color="#FFFFFF")
        self.safe_label.grid(row=0, column=0, columnspan=5,
                             padx=5, pady=10, sticky="n")

        # Center the X, Y, Z labels in the middle
        self.x_label_safe = ctk.CTkLabel(self.inner_padding_frame_safe, text="X : 00", font=(
            "Arial", 20, "bold"), text_color="#0048FF")
        self.x_label_safe.grid(row=1, column=0, padx=5, pady=10, sticky="nsew")

        self.y_label_safe = ctk.CTkLabel(self.inner_padding_frame_safe, text="Y : 00", font=(
            "Arial", 20, "bold"), text_color="#19FF00")
        self.y_label_safe.grid(row=1, column=1, padx=5, pady=10, sticky="nsew")

        self.z_label_safe = ctk.CTkLabel(self.inner_padding_frame_safe, text="Z : 00", font=(
            "Arial", 20, "bold"), text_color="#FF0000")
        self.z_label_safe.grid(row=1, column=2, padx=5, pady=10, sticky="nsew")

    def create_safe_mode_box(self):
        # Safe Mode box
        self.Safe_Mode = ctk.CTkFrame(
            self.telemetry_data_box_column1, height=600, width=400, fg_color="#2B2828", corner_radius=20)
        self.Safe_Mode.grid(row=1, column=3, padx=20, pady=10, sticky="nsew")

        # Add padding inside the frame using an inner container
        self.inner_padding_safe_mode = ctk.CTkFrame(
            self.Safe_Mode, fg_color="transparent")
        self.inner_padding_safe_mode.pack(
            fill="both", expand=True, padx=20, pady=20)

        # Configure grid to center elements
        self.inner_padding_safe_mode.grid_columnconfigure(0, weight=1)
        self.inner_padding_safe_mode.grid_rowconfigure((0, 1, 2), weight=1)

        # Add the "safe mode" label at the top center
        self.safe_mode_label = ctk.CTkLabel(self.inner_padding_safe_mode, text="Safe Mode", font=(
            "Arial", 24, "bold"), text_color="#FFFFFF")
        self.safe_mode_label.grid(
            row=0, column=0, padx=10, pady=(10, 20), sticky="n")

        # Create a small frame to group the Vertical and Horizontal labels
        self.safe_mode_values_frame = ctk.CTkFrame(
            self.inner_padding_safe_mode, fg_color="transparent")
        self.safe_mode_values_frame.grid(row=1, column=0, sticky="nsew")

        # Frame for vertical label and circle
        self.vertical_frame = ctk.CTkFrame(
            self.safe_mode_values_frame, fg_color="transparent")
        self.vertical_frame.pack(pady=20, padx=50, fill="x")

        self.vertical_label_safe_mode = ctk.CTkLabel(
            self.vertical_frame, text="Low Battery", font=("Arial", 20, "bold"), text_color="#FFFFFF")
        self.vertical_label_safe_mode.pack(side="left", padx=10)

        # Add a circle after the text
        self.vertical_circle = tk.Canvas(
            self.vertical_frame, width=40, height=40, bg="#2B2828", highlightthickness=0)
        self.battery_low_circle_id = self.vertical_circle.create_oval(
            5, 5, 35, 35, fill="#FFFFFF")  # Bigger red circle
        self.vertical_circle.pack(side="right", padx=10)

        # Frame for horizontal label and circle
        self.horizontal_frame = ctk.CTkFrame(
            self.safe_mode_values_frame, fg_color="transparent")
        self.horizontal_frame.pack(pady=20, fill="x", padx=50)

        self.horizontal_label_safe_mode = ctk.CTkLabel(
            self.horizontal_frame, text="Lost Link", font=("Arial", 20, "bold"), text_color="#FFFFFF")
        self.horizontal_label_safe_mode.pack(side="left", padx=10)

        # Add a circle after the text
        self.horizontal_circle = tk.Canvas(
            self.horizontal_frame, width=40, height=40, bg="#2B2828", highlightthickness=0)
        self.horizontal_circle_id = self.horizontal_circle.create_oval(
            5, 5, 35, 35, fill="#FFFFFF")  # Bigger green circle
        self.horizontal_circle.pack(side="right", padx=10)

    def create_control_mode_box(self):
        # Control Mode Column
        self.telemetry_data_box_column2 = ctk.CTkFrame(
            self.telemetry_data_box, height=500, width=200, fg_color="#E53935", corner_radius=1000)
        self.telemetry_data_box_column2.grid(
            row=1, column=1, sticky="nsew", padx=20, pady=20)

        # Inner padding frame
        self.inner_padding_control_mode = ctk.CTkFrame(
            self.telemetry_data_box_column2, fg_color="transparent")
        self.inner_padding_control_mode.pack(
            fill="both", expand=True, padx=20, pady=20)

        # Add "Control Modes" label at the top center
        self.control_modes_label = ctk.CTkLabel(self.inner_padding_control_mode, text="Control Modes", font=(
            "Arial", 24, "bold"), text_color="#FFFFFF")
        self.control_modes_label.pack(pady=(10, 20))

        # Add switch control below the label
        self.control_switch = ctk.CTkSwitch(
            self.inner_padding_control_mode,
            text="",
            font=("Arial", 30),
            corner_radius=20,
            switch_width=70,
            switch_height=30,
        )
        self.control_switch.pack(pady=20)

        # Add "Autonomous Mode
        # Add "Autonomous Mode" label in the center
        self.autonomous_mode_label = ctk.CTkLabel(
            self.inner_padding_control_mode, text="Autonomous Mode", font=("Arial", 20, "bold"), text_color="#FFFFFF")
        self.autonomous_mode_label.pack(pady=20)

    def create_camera_section(self):
        # Camera Box
        self.camera_box = ctk.CTkFrame(
            self.app, corner_radius=10, height=700, fg_color="transparent")
        self.camera_box.grid(row=3, column=0, sticky="ew", padx=20, pady=10)
        self.camera_box.grid_rowconfigure(1, weight=1)
        self.camera_box.grid_columnconfigure(2, weight=1)

        # Camera 1 Box (Fixed Width)
        self.camera_box_1 = ctk.CTkFrame(self.camera_box, height=750, width=int(
            self.screen_width * 0.45), fg_color="#2B2828", corner_radius=20)
        self.camera_box_1.grid(
            row=1, column=0, sticky="nsew", padx=20, pady=20)

        self.camera_box_1.grid_rowconfigure(0, weight=1)
        self.camera_box_1.grid_columnconfigure(0, weight=1)

        # Load and resize the image for Camera 1
        self.image_path = os.path.join(os.path.dirname(
            __file__), "assets/images/camera_1.png")
        self.image = ctk.CTkImage(light_image=Image.open(
            self.image_path), size=(int(self.screen_width * 0.45), 750))

        # # Add the image to a label and stretch it to fill the frame
        self.image_label = ctk.CTkLabel(
            self.camera_box_1, text="", image=self.image)
        self.image_label.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)

        # Camera 2 Box (Fixed Width)
        self.camera_box_2 = ctk.CTkFrame(self.camera_box, height=750, width=int(
            self.screen_width * 0.45), fg_color="#2B2828", corner_radius=20)
        self.camera_box_2.grid(
            row=1, column=3, sticky="nsew", padx=20, pady=20)

        self.camera_box_2.grid_rowconfigure(0, weight=1)
        self.camera_box_2.grid_columnconfigure(0, weight=1)

        # Load and resize the image for Camera 2
        self.image_path_2 = os.path.join(os.path.dirname(
            __file__), "assets/images/camera_2.png")
        self.image_2 = ctk.CTkImage(light_image=Image.open(
            self.image_path_2), size=(int(self.screen_width * 0.45), 750))

        # Add the image to a label and stretch it to fill the frame
        self.image_label_2 = ctk.CTkLabel(
            self.camera_box_2, image=self.image_2, text="")
        self.image_label_2.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)

    def run(self):
        # Run the app
        self.app.mainloop()


if __name__ == "__main__":
    app = FlightDataApp()
    app.run()
