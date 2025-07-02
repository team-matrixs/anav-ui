#!/usr/bin/env python3
import customtkinter as ctk
import os
import tkinter as tk
import threading
import cv2
from PIL import Image, ImageTk
import subprocess
import time
import numpy as np
import Xlib.display
import Xlib.X
from collections import deque
from subscriber import SensorSubscriber
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
import math

class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')
        self.autonomous_pub = self.create_publisher(Bool, '/autonomous_mode', 10)
        self.current_mode = False
        
    def set_mode(self, mode):
        """Set autonomous mode and publish to topic"""
        self.current_mode = mode
        msg = Bool()
        msg.data = self.current_mode
        self.autonomous_pub.publish(msg)
        self.get_logger().info(f"Autonomous mode set to: {self.current_mode}")


class RTABMapEmbed:
    def __init__(self, parent_frame):
        self.parent = parent_frame
        self.target_fps = 30
        self.capture_scale = 0.7
        self.buffer_size = 2
        self.last_frame_time = 0
        self.frame_buffer = deque(maxlen=self.buffer_size)
        
        # Create label for displaying RTAB-Map using grid
        self.image_label = ctk.CTkLabel(parent_frame, text="")
        self.image_label.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Configure the parent frame to expand the label
        parent_frame.grid_rowconfigure(0, weight=1)
        parent_frame.grid_columnconfigure(0, weight=1)
        
        # Process management
        self.capture_thread = None
        self.display_thread = None
        self.rtabmap_process = None
        self.running = False
        self.display = None
        self.rtabmap_window = None
        self.last_window_check = 0

    def start_capture(self):
        if not self.running:
            self.running = True
            
            # Launch RTAB-Map with optimized environment
            env = os.environ.copy()
            env.update({
                'DISPLAY': ':0',
                'QT_QUICK_BACKEND': 'software',
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'QT_AUTO_SCREEN_SCALE_FACTOR': '0'
            })
            
            # For ROS 2, the launch command would be different
            # self.rtabmap_process = subprocess.Popen(
            #     ["ros2 launch depthai_ros_driver rtabmap.launch.py"],
            #     env=env,
            #     stdout=subprocess.PIPE,
            #     stderr=subprocess.PIPE
            # )
            
            # Initialize X display in a separate thread
            self.display_thread = threading.Thread(target=self.init_display)
            self.display_thread.daemon = True
            self.display_thread.start()
            
            # Start capture thread
            self.capture_thread = threading.Thread(target=self.capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()

    def init_display(self):
        """Initialize X display connection"""
        self.display = Xlib.display.Display()
        time.sleep(1)  # Allow window to appear

    def find_rtabmap_window(self):
        """Find RTAB-Map window with minimal X calls"""
        if not self.display:
            return None
            
        try:
            root = self.display.screen().root
            window_ids = root.get_full_property(
                self.display.intern_atom('_NET_CLIENT_LIST'),
                Xlib.X.AnyPropertyType
            ).value
            
            for window_id in window_ids:
                window = self.display.create_resource_object('window', window_id)
                try:
                    if "rviz" in str(window.get_wm_name()).lower():
                        return window
                except:
                    continue
        except:
            pass
        return None

    def capture_loop(self):
        """Main capture loop for RTAB-Map window"""
        while self.running and rclpy.ok():
            try:
                # Find window periodically
                current_time = time.time()
                if not self.rtabmap_window or (current_time - self.last_window_check) > 1.0:
                    self.rtabmap_window = self.find_rtabmap_window()
                    self.last_window_check = current_time
                    if not self.rtabmap_window:
                        time.sleep(0.1)
                        continue
                
                # Get window geometry
                try:
                    geometry = self.rtabmap_window.get_geometry()
                except:
                    self.rtabmap_window = None
                    continue
                
                # Control frame rate
                target_frame_time = 1.0 / self.target_fps
                elapsed = current_time - self.last_frame_time
                if elapsed < target_frame_time:
                    time.sleep(max(0, target_frame_time - elapsed - 0.001))
                    continue
                
                # Capture window
                raw = self.rtabmap_window.get_image(
                    0, 0, geometry.width, geometry.height,
                    Xlib.X.ZPixmap, 0xffffffff
                )
                
                # Convert to numpy array
                frame = np.frombuffer(raw.data, dtype=np.uint8)
                frame = frame.reshape((geometry.height, geometry.width, 4))
                
                # Convert BGRA to BGR and resize
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                if self.capture_scale != 1.0:
                    frame = cv2.resize(frame, 
                                     (int(geometry.width * self.capture_scale),
                                      int(geometry.height * self.capture_scale)),
                                     interpolation=cv2.INTER_LINEAR)
                
                # Add to frame buffer
                self.frame_buffer.append(frame)
                
                # Schedule display update
                self.parent.after(0, self.update_display)
                
                self.last_frame_time = current_time
                
            except Exception as e:
                print(f"RTAB-Map capture error: {str(e)}")
                time.sleep(0.1)

    def update_display(self):
        """Update the display with the latest frame"""
        if self.frame_buffer:
            frame = self.frame_buffer[-1]
            
            try:
                height, width = frame.shape[:2]
                image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                ctk_image = ctk.CTkImage(light_image=image, size=(width, height))
                self.image_label.configure(image=ctk_image)
            except Exception as e:
                print(f"RTAB-Map display error: {str(e)}")

    def stop_capture(self):
        """Stop the RTAB-Map capture"""
        if self.running:
            self.running = False
            if self.rtabmap_process:
                self.rtabmap_process.terminate()
                try:
                    self.rtabmap_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.rtabmap_process.kill()
            if self.display:
                self.display.close()

class FlightDataApp():
    def __init__(self):
        self.app = ctk.CTk()
        self.app.title("Flight Data - (Team Matrix)")
        logo_path = os.path.join(os.getcwd(), "assets", "images", "logo.png")
        if os.path.exists(logo_path):
            icon_image = tk.PhotoImage(file=logo_path)
            self.app.wm_iconphoto(True, icon_image)
        self.app.configure(fg_color="#221F1F")
        # Configure grid for responsiveness
        self.app.grid_columnconfigure(0, weight=1)
        self.app.grid_rowconfigure(1, weight=1)
        
        # Initialize ROS 2
        rclpy.init()
        
        # Create ROS 2 nodes
        self.autonomous_controller = AutonomousController()
        
        # Initialize UI components
        self.create_nav_bar()
        self.create_main_content()
        
        # Create ROS 2 subscribers
        self.imu_node = SensorSubscriber(
            self.app, self.x_label, self.y_label, self.z_label, 
            self.status_label2, self.status_label4, self.vertical_circle, 
            self.battery_low_circle_id, self.horizontal_circle, 
            self.horizontal_circle_id, self.vertical_label, 
            self.horizontal_label, self.height_label, self.x_label_safe, 
            self.y_label_safe, self.z_label_safe)
        

        # Start ROS nodes in separate threads
        self.ros_thread = threading.Thread(target=self.run_ros_nodes)
        self.ros_thread.daemon = True
        self.ros_thread.start()

        self.rtabmap_embed = RTABMapEmbed(self.camera_box)
        self.rtabmap_embed.start_capture()

        # Handle app close event
        self.app.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Bind resize event
        self.app.bind("<Configure>", self.on_window_resize)

    def update_camera_frame(self, cv_image):
        try:
            # Convert OpenCV image to PIL format
            img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(img)
            
            # Set FIXED width and height (adjust as needed)
            fixed_width = 800  # Example: 640px width
            fixed_height = 480  # Example: 480px height (4:3 aspect ratio)
            
            # Resize image to exact dimensions (may stretch if aspect ratio differs)
            resized_img = pil_img.resize((fixed_width, fixed_height), Image.LANCZOS)
            tk_img = ImageTk.PhotoImage(image=resized_img)
            
            # Update display
            self.image_label.configure(image=tk_img)
            self.image_label.image = tk_img  # Keep reference
    
        except Exception as e:
            print(f"Error updating camera frame: {str(e)}")

    def run_ros_nodes(self):
        """Run ROS nodes in separate threads"""
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.autonomous_controller)
        executor.add_node(self.imu_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            self.autonomous_controller.destroy_node()
            self.imu_node.destroy_node()

    def on_window_resize(self, event):
        """Handle window resize events to maintain aspect ratio"""
        if not self.app.attributes('-fullscreen'):
            # Calculate target dimensions based on 16:9 aspect ratio
            width = self.app.winfo_width()
            target_height = width * 9 // 16
            
            # Don't allow window to be taller than screen
            screen_height = self.app.winfo_screenheight()
            if target_height > screen_height:
                target_height = screen_height
                width = target_height * 16 // 9
            
            # Resize window to maintain aspect ratio
            if event.widget == self.app:
                self.app.geometry("%dx%d" % (width, target_height))

    def on_close(self):
        """Handles application exit gracefully."""
        print("Shutting down...")
        
        # Stop RTAB-Map capture
        self.rtabmap_embed.stop_capture()
        
        # Shutdown ROS 2
        rclpy.shutdown()
        
        # Close UI
        self.app.destroy()

    def create_nav_bar(self):
        """Create the navigation bar at the top of the window"""
        # Navigation bar
        self.nav_bar = ctk.CTkFrame(self.app, fg_color="#E53935", height=80)
        self.nav_bar.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)
        self.nav_bar.grid_columnconfigure(0, weight=1)
        self.nav_bar.grid_columnconfigure(1, weight=1)
        self.nav_bar.grid_rowconfigure(0, weight=1)
        
        # Navigation heading
        self.navbar_heading = ctk.CTkLabel(
            self.nav_bar, text="Flight Data", 
            font=("Arial", 30, "bold"), text_color="#ffffff")
        self.navbar_heading.grid(row=0, column=0, sticky="w", padx=30, pady=0)
        
        # Status box
        self.create_status_box()

    def create_status_box(self):
        """Create the status indicators in the nav bar"""
        self.navbar_status_box = ctk.CTkFrame(
            self.nav_bar, height=60, fg_color="#2B2828")
        self.navbar_status_box.grid(row=0, column=1, sticky="e", padx=20, pady=0)
        
        # Use a single row with multiple columns for better responsiveness
        self.navbar_status_box.grid_columnconfigure(0, weight=1)
        self.navbar_status_box.grid_columnconfigure(1, weight=1)
        self.navbar_status_box.grid_rowconfigure(0, weight=1)
        
        # Battery Status
        self.status_label1 = ctk.CTkLabel(
            self.navbar_status_box, text="Battery: ", 
            font=("Arial", 18, "bold"), text_color="#FFFFFF")
        self.status_label1.grid(row=0, column=0, sticky="e", padx=(25, 5), pady=0)
        
        self.status_label2 = ctk.CTkLabel(
            self.navbar_status_box, text="0%", 
            font=("Arial", 18, "bold"), text_color="#FF0000")
        self.status_label2.grid(row=0, column=1, sticky="w", padx=(0, 20), pady=0)
        
        # Battery Health
        self.status_label3 = ctk.CTkLabel(
            self.navbar_status_box, text="Health: ", 
            font=("Arial", 18, "bold"), text_color="#FFFFFF")
        self.status_label3.grid(row=0, column=2, sticky="e", padx=(0, 5), pady=0)
        
        self.status_label4 = ctk.CTkLabel(
            self.navbar_status_box, text="Normal", 
            font=("Arial", 18, "bold"), text_color="#FF0000")
        self.status_label4.grid(row=0, column=3, sticky="w", padx=(0, 20), pady=0)

    def create_main_content(self):
        """Create the main content area below the nav bar"""
        self.main_content = ctk.CTkFrame(self.app, fg_color="transparent")
        self.main_content.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
        self.main_content.grid_columnconfigure(0, weight=1)
        self.main_content.grid_rowconfigure(1, weight=1)
        
        # Telemetry section
        self.create_telemetry_section()
        
        # Camera section
        self.create_camera_section()

    def create_telemetry_section(self):
        """Create the telemetry information section"""
        # Telemetry Heading
        self.telemetry_heading = ctk.CTkLabel(
            self.main_content, text="Telemetry Information", 
            font=("Arial", 28, "bold"), fg_color="transparent", text_color="#FFFFFF")
        self.telemetry_heading.grid(
            row=0, column=0, padx=20, pady=(0, 10), sticky="w")
        
        # Telemetry Data Container
        self.telemetry_container = ctk.CTkFrame(
            self.main_content, fg_color="transparent")
        self.telemetry_container.grid(
            row=1, column=0, sticky="nsew", padx=0, pady=0)
        
        # Configure grid for telemetry boxes (4 columns)
        for i in range(4):
            self.telemetry_container.grid_columnconfigure(i, weight=1, uniform="cols")
        self.telemetry_container.grid_rowconfigure(0, weight=1)
        
        # Create telemetry boxes with uniform sizing
        self.create_angular_velocity_box()
        self.create_velocity_box()
        self.create_safe_site_box()
        self.create_safe_mode_box()
        
        # Control Mode box (special case, goes to the side)
        # self.create_control_mode_box()

    def create_angular_velocity_box(self):
        """Create the orientation/angular velocity box"""
        self.Angular_velocity = ctk.CTkFrame(
            self.telemetry_container, fg_color="#2B2828", corner_radius=15)
        self.Angular_velocity.grid(
            row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.Angular_velocity.grid_columnconfigure(0, weight=1)
        self.Angular_velocity.grid_rowconfigure(1, weight=1)
        
        # Title
        self.angular_velocity_label = ctk.CTkLabel(
            self.Angular_velocity, text="Orientation", 
            font=("Arial", 24, "bold"), text_color="#FFFFFF")
        self.angular_velocity_label.grid(
            row=0, column=0, padx=10, pady=(10, 5), sticky="n")
        
        # Data frame
        self.orientation_data_frame = ctk.CTkFrame(
            self.Angular_velocity, fg_color="transparent")
        self.orientation_data_frame.grid(
            row=1, column=0, sticky="nsew", padx=10, pady=10)
        
        # Configure grid for data labels
        self.orientation_data_frame.grid_columnconfigure(0, weight=1)
        for i in range(3):
            self.orientation_data_frame.grid_rowconfigure(i, weight=1)
        
        # Data labels
        self.x_label = ctk.CTkLabel(
            self.orientation_data_frame, text="Roll : 00 deg", 
            font=("Arial", 18, "bold"), text_color="#0048FF")
        self.x_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        self.y_label = ctk.CTkLabel(
            self.orientation_data_frame, text="Pitch : 00 deg", 
            font=("Arial", 18, "bold"), text_color="#19FF00")
        self.y_label.grid(row=1, column=0, padx=5, pady=5, sticky="w")
        
        self.z_label = ctk.CTkLabel(
            self.orientation_data_frame, text="Yaw : 00 deg", 
            font=("Arial", 18, "bold"), text_color="#FF0000")
        self.z_label.grid(row=2, column=0, padx=5, pady=5, sticky="w")

    def create_velocity_box(self):
        """Create the velocity information box"""
        self.Velocity = ctk.CTkFrame(
            self.telemetry_container, fg_color="#2B2828", corner_radius=15)
        self.Velocity.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        self.Velocity.grid_columnconfigure(0, weight=1)
        self.Velocity.grid_rowconfigure(1, weight=1)
        
        # Title
        self.velocity_label = ctk.CTkLabel(
            self.Velocity, text="Velocity & Height", 
            font=("Arial", 24, "bold"), text_color="#FFFFFF")
        self.velocity_label.grid(
            row=0, column=0, padx=10, pady=(10, 5), sticky="n")
        
        # Data frame
        self.velocity_data_frame = ctk.CTkFrame(
            self.Velocity, fg_color="transparent")
        self.velocity_data_frame.grid(
            row=1, column=0, sticky="nsew", padx=10, pady=10)
        
        # Configure grid for data labels
        self.velocity_data_frame.grid_columnconfigure(0, weight=1)
        for i in range(3):
            self.velocity_data_frame.grid_rowconfigure(i, weight=1)
        
        # Data labels
        self.vertical_label = ctk.CTkLabel(
            self.velocity_data_frame, text="Vertical : 00 m/s", 
            font=("Arial", 18, "bold"), text_color="#FF0000")
        self.vertical_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        self.horizontal_label = ctk.CTkLabel(
            self.velocity_data_frame, text="Horizontal : 00 m/s", 
            font=("Arial", 18, "bold"), text_color="#11FF00")
        self.horizontal_label.grid(row=1, column=0, padx=5, pady=5, sticky="w")
        
        self.height_label = ctk.CTkLabel(
            self.velocity_data_frame, text="Height : 00 cm", 
            font=("Arial", 18, "bold"), text_color="#FFFFFF")
        self.height_label.grid(row=2, column=0, padx=5, pady=5, sticky="w")

    def create_safe_site_box(self):
        """Create the safe site coordinates box"""
        self.Safe_Site = ctk.CTkFrame(
            self.telemetry_container, fg_color="#2B2828", corner_radius=15)
        self.Safe_Site.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")
        self.Safe_Site.grid_columnconfigure(0, weight=1)
        self.Safe_Site.grid_rowconfigure(1, weight=1)
        
        # Title
        self.safe_label = ctk.CTkLabel(
            self.Safe_Site, text="Safe Site Co-Ordinates", 
            font=("Arial", 24, "bold"), text_color="#FFFFFF")
        self.safe_label.grid(
            row=0, column=0, padx=10, pady=(10, 5), sticky="n")
        
        # Data frame
        self.safe_site_data_frame = ctk.CTkFrame(
            self.Safe_Site, fg_color="transparent")
        self.safe_site_data_frame.grid(
            row=1, column=0, sticky="nsew", padx=10, pady=10)
        
        # Configure grid for data labels
        self.safe_site_data_frame.grid_columnconfigure(0, weight=1)
        for i in range(3):
            self.safe_site_data_frame.grid_rowconfigure(i, weight=1)
        
        # Data labels
        self.x_label_safe = ctk.CTkLabel(
            self.safe_site_data_frame, text="1 : 00", 
            font=("Arial", 18, "bold"), text_color="#0048FF")
        self.x_label_safe.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        self.y_label_safe = ctk.CTkLabel(
            self.safe_site_data_frame, text="2 : 00", 
            font=("Arial", 18, "bold"), text_color="#19FF00")
        self.y_label_safe.grid(row=1, column=0, padx=5, pady=5, sticky="w")
        
        self.z_label_safe = ctk.CTkLabel(
            self.safe_site_data_frame, text="3 : 00", 
            font=("Arial", 18, "bold"), text_color="#FF0000")
        self.z_label_safe.grid(row=2, column=0, padx=5, pady=5, sticky="w")

    def create_safe_mode_box(self):
        """Create the safe mode indicators box"""
        self.Safe_Mode = ctk.CTkFrame(
            self.telemetry_container, fg_color="#2B2828", corner_radius=15)
        self.Safe_Mode.grid(row=0, column=3, padx=5, pady=5, sticky="nsew")
        self.Safe_Mode.grid_columnconfigure(0, weight=1)
        self.Safe_Mode.grid_rowconfigure(1, weight=1)
        
        # Title
        self.safe_mode_label = ctk.CTkLabel(
            self.Safe_Mode, text="Safe Mode", 
            font=("Arial", 24, "bold"), text_color="#FFFFFF")
        self.safe_mode_label.grid(
            row=0, column=0, padx=10, pady=(10, 5), sticky="n")
        
        # Data frame
        self.safe_mode_data_frame = ctk.CTkFrame(
            self.Safe_Mode, fg_color="transparent")
        self.safe_mode_data_frame.grid(
            row=1, column=0, sticky="nsew", padx=10, pady=10)
        
        # Configure grid for indicators
        self.safe_mode_data_frame.grid_columnconfigure(0, weight=1)
        for i in range(2):
            self.safe_mode_data_frame.grid_rowconfigure(i, weight=1)
        
        # Low Battery indicator
        self.low_battery_frame = ctk.CTkFrame(
            self.safe_mode_data_frame, fg_color="transparent")
        self.low_battery_frame.grid(
            row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        self.low_battery_label = ctk.CTkLabel(
            self.low_battery_frame, text="Low Battery", 
            font=("Arial", 18, "bold"), text_color="#FFFFFF")
        self.low_battery_label.pack(side="left", padx=5)
        
        self.vertical_circle = tk.Canvas(
            self.low_battery_frame, width=30, height=30, 
            bg="#2B2828", highlightthickness=0)
        self.battery_low_circle_id = self.vertical_circle.create_oval(
            5, 5, 25, 25, fill="#FFFFFF")
        self.vertical_circle.pack(side="right", padx=5)
        
        # Lost Link indicator
        self.lost_link_frame = ctk.CTkFrame(
            self.safe_mode_data_frame, fg_color="transparent")
        self.lost_link_frame.grid(
            row=1, column=0, sticky="nsew", padx=5, pady=5)
        
        self.lost_link_label = ctk.CTkLabel(
            self.lost_link_frame, text="Lost Link", 
            font=("Arial", 18, "bold"), text_color="#FFFFFF")
        self.lost_link_label.pack(side="left", padx=5)
        
        self.horizontal_circle = tk.Canvas(
            self.lost_link_frame, width=30, height=30, 
            bg="#2B2828", highlightthickness=0)
        self.horizontal_circle_id = self.horizontal_circle.create_oval(
            5, 5, 25, 25, fill="#FFFFFF")
        self.horizontal_circle.pack(side="right", padx=5)

    def create_control_mode_box(self):
        """Create the control mode selector box"""
        # This box spans a new row below the other telemetry boxes
        self.Control_Mode = ctk.CTkFrame(
            self.telemetry_container, fg_color="#E53935", corner_radius=15)
        self.Control_Mode.grid(
            row=1, column=0, columnspan=4, padx=5, pady=5, 
            sticky="nsew")
        self.Control_Mode.grid_columnconfigure(0, weight=1)
        self.Control_Mode.grid_rowconfigure(1, weight=1)
        
        # Title
        self.control_modes_label = ctk.CTkLabel(
            self.Control_Mode, text="Control Modes", 
            font=("Arial", 24, "bold"), text_color="#FFFFFF")
        self.control_modes_label.grid(
            row=0, column=0, padx=10, pady=(10, 5), sticky="n")
        
        # Control frame
        self.control_frame = ctk.CTkFrame(
            self.Control_Mode, fg_color="transparent")
        self.control_frame.grid(
            row=1, column=0, sticky="nsew", padx=10, pady=10)
        
        # Switch and label
        self.control_switch = ctk.CTkSwitch(
            self.control_frame,
            text="Autonomous Mode",
            text_color="#ffffff",
            font=("Arial", 18),
            corner_radius=20,
            switch_width=60,
            switch_height=30,
            command=self.on_autonomous_switch_changed
        )
        self.control_switch.pack(pady=10)

    def create_camera_section(self):
        """Create the camera display section with full-width RTAB-Map view"""
        self.camera_section = ctk.CTkFrame(
            self.main_content, fg_color="transparent")
        self.camera_section.grid(
            row=2, column=0, sticky="nsew", padx=0, pady=(10, 0))
        self.camera_section.grid_columnconfigure(0, weight=1)
        self.camera_section.grid_rowconfigure(0, weight=1)
        
        # Camera Heading
        self.camera_heading = ctk.CTkLabel(
        self.camera_section, text="Elevation Map View", 
            font=("Arial", 24, "bold"), fg_color="transparent", text_color="#FFFFFF")
        self.camera_heading.grid(
        row=0, column=0, padx=20, pady=(0, 10), sticky="w")
        
        # Single camera container with fixed height
        self.camera_box = ctk.CTkFrame(
            self.camera_section, 
            fg_color="#2B2828", 
            corner_radius=15,
            height=450)  # Fixed height of 500 pixels
        self.camera_box.grid(
            row=1, column=0, 
            sticky="nsew", 
            padx=5, pady=5)
        self.camera_box.grid_propagate(False)  # Prevent resizing
        self.camera_box.grid_rowconfigure(0, weight=1)
        self.camera_box.grid_columnconfigure(0, weight=1)
        
        # Create image label for camera feed
        self.image_label = ctk.CTkLabel(self.camera_box, text="")
        self.image_label.grid(row=0, column=0, sticky="nsew")

    def on_autonomous_switch_changed(self):
        """Handle autonomous mode switch changes"""
        mode = self.control_switch.get()
        self.autonomous_controller.set_mode(mode)
        
        # Visual feedback
        if mode:
            self.control_switch.configure(text="Autonomous Mode: ON")
        else:
            self.control_switch.configure(text="Autonomous Mode: OFF")
        
    def run(self):
        """Run the application"""
        self.app.mainloop()

if __name__ == "__main__":
    app = FlightDataApp()
    app.run()