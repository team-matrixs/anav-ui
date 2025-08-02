# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
# Modified to be responsive for full screen

from PyQt5 import QtCore, QtGui, QtWidgets
from new_sub import SensorSubscriber
import cv2
import numpy as np
from collections import deque
import time
import threading
import subprocess
import os
import Xlib.display
from PIL import Image, ImageQt
from PyQt5.QtCore import QTimer
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node

class AutonomousModePublisher:
    def __init__(self, node):
        self.node = node
        self.publisher = self.node.create_publisher(Bool, 'autonomous_mode', 10)
        
    def publish_mode(self, mode):
        msg = Bool()
        msg.data = mode
        self.publisher.publish(msg)
        self.node.get_logger().info(f'Publishing autonomous mode: {mode}')
        
        
        
class RTABMapEmbed:
    def __init__(self, parent_widget):
        self.parent = parent_widget
        self.target_fps = 30
        self.capture_scale = 0.7
        self.buffer_size = 2
        self.last_frame_time = 0
        self.frame_buffer = deque(maxlen=self.buffer_size)
        self.window_check_interval = 1.0  # seconds
        self.max_retries = 3
        self.retry_count = 0
        self.window_active = False  # Track if we have an active window
        
        # Create label for displaying RTAB-Map
        self.image_label = QtWidgets.QLabel(parent_widget)
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: black;")
        self.image_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        
        # Add to parent layout if it exists, otherwise just set as child
        if hasattr(parent_widget, 'layout'):
            parent_widget.layout().addWidget(self.image_label)
        
        # Process management
        self.capture_thread = None
        self.display_thread = None
        self.rtabmap_process = None
        self.running = False
        self.display = None
        self.rtabmap_window = None
        self.last_window_check = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)

    def start_capture(self):
        if not self.running:
            self.running = True
            self.launch_rtabmap()
            
            # Initialize X display in a separate thread
            self.display_thread = threading.Thread(target=self.init_display)
            self.display_thread.daemon = True
            self.display_thread.start()
            
            # Start capture thread
            self.capture_thread = threading.Thread(target=self.capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            
            # Start display update timer
            self.timer.start(int(1000 / self.target_fps))

    def launch_rtabmap(self):
        """Launch RTAB-Map process with optimized environment"""
        if self.rtabmap_process and self.rtabmap_process.poll() is None:
            return  # Process is already running
            
        env = os.environ.copy()
        env.update({
            'DISPLAY': ':0',
            'QT_QUICK_BACKEND': 'software',
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_AUTO_SCREEN_SCALE_FACTOR': '0'
        })
        
        try:
            self.rtabmap_process = subprocess.Popen(
                ["ros2", "launch", "rtabmap_launch", "rtabmap.launch.py"],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.retry_count = 0  # Reset retry counter on successful launch
        except Exception as e:
            print(f"Failed to launch RTAB-Map: {str(e)}")
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                time.sleep(2)
                self.launch_rtabmap()
            else:
                print("Max retries reached for RTAB-Map launch")
                self.running = False

    def init_display(self):
        """Initialize X display connection"""
        try:
            self.display = Xlib.display.Display()
            time.sleep(1)  # Allow window to appear
        except Exception as e:
            print(f"Failed to initialize X display: {str(e)}")
            self.running = False

    def find_rtabmap_window(self):
        """Find RTAB-Map or RViz window with minimal X calls"""
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
                    name = str(window.get_wm_name()).lower()
                    if "rtab-map" in name or "rtabmap" in name or "rviz" in name:
                        return window
                except:
                    continue
        except Exception as e:
            print(f"Window search error: {str(e)}")
        return None

    def capture_loop(self):
        """Main capture loop for RTAB-Map window"""
        while self.running:
            try:
                # Check if RTAB-Map process is still running
                if self.rtabmap_process and self.rtabmap_process.poll() is not None:
                    print("RTAB-Map process terminated, restarting...")
                    self.launch_rtabmap()
                    time.sleep(2)  # Give time for restart
                    continue
                
                # Find window periodically
                current_time = time.time()
                if not self.rtabmap_window or (current_time - self.last_window_check) > self.window_check_interval:
                    previous_window = self.rtabmap_window
                    self.rtabmap_window = self.find_rtabmap_window()
                    self.last_window_check = current_time
                    
                    # Update window active status
                    if not self.rtabmap_window:
                        self.window_active = False
                        time.sleep(0.1)
                        continue
                    else:
                        self.window_active = True
                
                # Get window geometry
                try:
                    geometry = self.rtabmap_window.get_geometry()
                except:
                    self.rtabmap_window = None
                    self.window_active = False
                    continue
                
                # Control frame rate
                target_frame_time = 1.0 / self.target_fps
                elapsed = current_time - self.last_frame_time
                if elapsed < target_frame_time:
                    time.sleep(max(0, target_frame_time - elapsed - 0.001))
                    continue
                
                # Capture window
                try:
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
                    
                    self.last_frame_time = current_time
                except:
                    self.rtabmap_window = None
                    self.window_active = False
                    continue
                
            except Exception as e:
                print(f"RTAB-Map capture error: {str(e)}")
                self.window_active = False
                time.sleep(0.1)

    def update_display(self):
        """Update the display with the latest frame or show blank if no window"""
        if not self.window_active or not self.frame_buffer:
            # Show blank black screen
            blank_pixmap = QtGui.QPixmap(self.image_label.size())
            blank_pixmap.fill(QtGui.QColor(0, 0, 0))
            self.image_label.setPixmap(blank_pixmap)
            return
            
        try:
            frame = self.frame_buffer[-1]
            
            # Convert numpy array to QImage
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_img = QtGui.QImage(frame.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888).rgbSwapped()
            
            # Convert to QPixmap and display
            pixmap = QtGui.QPixmap.fromImage(q_img)
            self.image_label.setPixmap(pixmap.scaled(
                self.image_label.size(), 
                QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation
            ))
        except Exception as e:
            print(f"RTAB-Map display error: {str(e)}")
            # Show blank screen on error
            blank_pixmap = QtGui.QPixmap(self.image_label.size())
            blank_pixmap.fill(QtGui.QColor(0, 0, 0))
            self.image_label.setPixmap(blank_pixmap)

    def stop_capture(self):
        """Stop the RTAB-Map capture"""
        if self.running:
            self.running = False
            self.timer.stop()
            if self.rtabmap_process:
                self.rtabmap_process.terminate()
                try:
                    self.rtabmap_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.rtabmap_process.kill()
            if self.display:
                self.display.close()
                
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        """Initialize and set up the main window UI components."""
        
        # ==============================================
        # Main Window Configuration
        # ==============================================
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        
        # Get screen dimensions for responsive design
        screen = QtWidgets.QApplication.desktop().screenGeometry()
        screen_width = screen.width()
        screen_height = screen.height()
        
        # Set window to full screen with dark background
        MainWindow.resize(screen_width, screen_height)
        MainWindow.setStyleSheet("background-color: #07090B")
        MainWindow.setAnimated(True)
        
        # Calculate scaling factors based on original design (1103x748)
        self.scale_x = screen_width / 1103.0
        self.scale_y = screen_height / 748.0
        
        # Main container widget
        self.widget = QtWidgets.QWidget(MainWindow)
        self.widget.setGeometry(0, 0, screen_width, screen_height)
        self.widget.setObjectName("widget")
        
        # ==============================================
        # Bottom Panel Section
        # ==============================================
        # Bottom panel - responsive positioning and sizing
        bottom_panel_height = int(251 * self.scale_y)
        bottom_panel_width = int(770 * self.scale_x)
        bottom_panel_x = int(20 * self.scale_x)
        bottom_panel_y = screen_height - bottom_panel_height - int(50 * self.scale_y)
        
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setGeometry(QtCore.QRect(bottom_panel_x, bottom_panel_y, bottom_panel_width, bottom_panel_height))
        self.widget_2.setStyleSheet("border-radius: 10px;\n"
                                    "background-color: #101419;")
        self.widget_2.setObjectName("widget_2")
        
        # --------------------------------------------------
        # Left Orientation Widget (in bottom panel)
        # --------------------------------------------------
        self.widget_5 = QtWidgets.QWidget(self.widget_2)
        self.widget_5.setGeometry(QtCore.QRect(int(20 * self.scale_x), int(30 * self.scale_y), 
                                int(181 * self.scale_x), int(201 * self.scale_y)))
        self.widget_5.setStyleSheet("background-color: #1D1D1D;\n"
                                    "border: 1px solid #B9B9B9;")
        self.widget_5.setObjectName("widget_5")
        
        # Orientation labels
        self.label_2 = QtWidgets.QLabel(self.widget_5)
        self.label_2.setGeometry(QtCore.QRect(int(40 * self.scale_x), 0, int(111 * self.scale_x), int(41 * self.scale_y)))
        self.label_2.setStyleSheet("border: 0px;\n"
                                    "font-weight: bold;\n"
                                    "color: #A4A4A5;\n"
                                    f"font-size: {int(20 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_2.setObjectName("label_2")
        
        # X, Y, Z orientation values
        self.label_7 = QtWidgets.QLabel(self.widget_5)
        self.label_7.setGeometry(QtCore.QRect(int(40 * self.scale_x), int(40 * self.scale_y), 
                                int(141 * self.scale_x), int(41 * self.scale_y)))
        self.label_7.setStyleSheet("border: 0px;\n"
                                    "font-weight: 600;\n"
                                    "color: white;\n"
                                    f"font-size: {int(28 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_7.setObjectName("label_7")
        
        self.label_8 = QtWidgets.QLabel(self.widget_5)
        self.label_8.setGeometry(QtCore.QRect(int(40 * self.scale_x), int(90 * self.scale_y), 
                                int(141 * self.scale_x), int(41 * self.scale_y)))
        self.label_8.setStyleSheet("border: 0px;\n"
                                    "font-weight: 600;\n"
                                    "color: white;\n"
                                    f"font-size: {int(28 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_8.setObjectName("label_8")
        
        self.label_9 = QtWidgets.QLabel(self.widget_5)
        self.label_9.setGeometry(QtCore.QRect(int(40 * self.scale_x), int(140 * self.scale_y), 
                                int(141 * self.scale_x), int(41 * self.scale_y)))
        self.label_9.setStyleSheet("border: 0px;\n"
                                    "font-weight: 600;\n"
                                    "color: white;\n"
                                    f"font-size: {int(28 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_9.setObjectName("label_9")
        
        # --------------------------------------------------
        # Center Safety Widget (in bottom panel)
        # --------------------------------------------------
        self.widget_6 = QtWidgets.QWidget(self.widget_2)
        self.widget_6.setGeometry(QtCore.QRect(int(230 * self.scale_x), int(110 * self.scale_y), 
                                int(311 * self.scale_x), int(121 * self.scale_y)))
        self.widget_6.setStyleSheet("background-color: #1D1D1D;\n"
                                    "border: 1px solid #B9B9B9;")
        self.widget_6.setObjectName("widget_6")
        
        # Safety status labels
        self.label_3 = QtWidgets.QLabel(self.widget_6)
        self.label_3.setGeometry(QtCore.QRect(int(20 * self.scale_x), 0, int(111 * self.scale_x), int(41 * self.scale_y)))
        self.label_3.setStyleSheet("border: 0px;\n"
                                    "font-weight: bold;\n"
                                    "color: #A4A4A5;\n"
                                    f"font-size: {int(18 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_3.setObjectName("label_3")
        
        self.label_4 = QtWidgets.QLabel(self.widget_6)
        self.label_4.setGeometry(QtCore.QRect(int(20 * self.scale_x), int(35 * self.scale_y), 
                                            int(141 * self.scale_x), int(41 * self.scale_y)))
        self.label_4.setStyleSheet("border: 0px;\n"
                                    "font-weight: 550;\n"
                                    "color: white;\n"
                                    f"font-size: {int(23 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_4.setObjectName("label_4")
        
        self.label_5 = QtWidgets.QLabel(self.widget_6)
        self.label_5.setGeometry(QtCore.QRect(int(20 * self.scale_x), int(71 * self.scale_y), 
                                            int(141 * self.scale_x), int(41 * self.scale_y)))
        self.label_5.setStyleSheet("border: 0px;\n"
                                    "font-weight: 550;\n"
                                    "color: white;\n"
                                    f"font-size: {int(23 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_5.setObjectName("label_5")
        
        # Status indicator dots
        self.widget_12 = QtWidgets.QWidget(self.widget_6)
        self.widget_12.setGeometry(QtCore.QRect(int(270 * self.scale_x), int(47 * self.scale_y), 
                                            int(18 * self.scale_x), int(21 * self.scale_y)))
        self.widget_12.setStyleSheet("border-radius: 12px;\n"
                                    "border: none;\n"
                                    "background-color: #A4A4A5;")
        self.widget_12.setObjectName("widget_12")
        
        self.widget_13 = QtWidgets.QWidget(self.widget_6)
        self.widget_13.setGeometry(QtCore.QRect(int(270 * self.scale_x), int(85 * self.scale_y), 
                                            int(18 * self.scale_x), int(21 * self.scale_y)))
        self.widget_13.setStyleSheet("border-radius: 12px;\n"
                                    "border: none;\n"
                                    "background-color: #A4A4A5;")
        self.widget_13.setObjectName("widget_13")
        
        # --------------------------------------------------
        # Right Velocity Widget (in bottom panel)
        # --------------------------------------------------
        self.widget_7 = QtWidgets.QWidget(self.widget_2)
        self.widget_7.setGeometry(QtCore.QRect(int(570 * self.scale_x), int(30 * self.scale_y), 
                                int(181 * self.scale_x), int(201 * self.scale_y)))
        self.widget_7.setStyleSheet("background-color: #1D1D1D;\n"
                                    "border: 1px solid #B9B9B9;")
        self.widget_7.setObjectName("widget_7")
        
        # Velocity labels
        self.label_6 = QtWidgets.QLabel(self.widget_7)
        self.label_6.setGeometry(QtCore.QRect(int(50 * self.scale_x), 0, int(111 * self.scale_x), int(41 * self.scale_y)))
        self.label_6.setStyleSheet("border: 0px;\n"
                                    "font-weight: bold;\n"
                                    "color: #A4A4A5;\n"
                                    f"font-size: {int(20 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_6.setObjectName("label_6")
        
        # X, Y, Z velocity values
        self.label_10 = QtWidgets.QLabel(self.widget_7)
        self.label_10.setGeometry(QtCore.QRect(int(36 * self.scale_x), int(40 * self.scale_y), 
                                int(141 * self.scale_x), int(41 * self.scale_y)))
        self.label_10.setStyleSheet("border: 0px;\n"
                                    "font-weight: 600;\n"
                                    "color: white;\n"
                                    f"font-size: {int(28 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_10.setObjectName("label_10")
        
        self.label_11 = QtWidgets.QLabel(self.widget_7)
        self.label_11.setGeometry(QtCore.QRect(int(36 * self.scale_x), int(90 * self.scale_y), 
                                int(141 * self.scale_x), int(41 * self.scale_y)))
        self.label_11.setStyleSheet("border: 0px;\n"
                                    "font-weight: 600;\n"
                                    "color: white;\n"
                                    f"font-size: {int(28 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_11.setObjectName("label_11")
        
        self.label_12 = QtWidgets.QLabel(self.widget_7)
        self.label_12.setGeometry(QtCore.QRect(int(36 * self.scale_x), int(140 * self.scale_y), 
                                int(141 * self.scale_x), int(41 * self.scale_y)))
        self.label_12.setStyleSheet("border: 0px;\n"
                                    "font-weight: 600;\n"
                                    "color: white;\n"
                                    f"font-size: {int(28 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_12.setObjectName("label_12")
        
        # Logo in bottom panel
        self.label_23 = QtWidgets.QLabel(self.widget_2)
        self.label_23.setGeometry(QtCore.QRect(int(350 * self.scale_x), int(20 * self.scale_y), 
                                int(78 * self.scale_x), int(81 * self.scale_y)))
        self.label_23.setText("")
        self.label_23.setPixmap(QtGui.QPixmap("assets/images/logo.png"))
        self.label_23.setScaledContents(True)
        self.label_23.setObjectName("label_23")
        
        # ==============================================
        # Right Panel Section
        # ==============================================
        # Right panel - responsive positioning and sizing
        right_panel_width = int(271 * self.scale_x)
        right_panel_x = screen_width - right_panel_width - int(20 * self.scale_x)
        
        self.widget_3 = QtWidgets.QWidget(self.widget)
        self.widget_3.setGeometry(QtCore.QRect(right_panel_x, int(10 * self.scale_y), 
                                right_panel_width, int(721 * self.scale_y)))
        self.widget_3.setStyleSheet("border-radius: 10px;\n"
                                    "background-color: #101419;")
        self.widget_3.setObjectName("widget_3")
        
        # --------------------------------------------------
        # Safe Coordinates Widget (in right panel)
        # --------------------------------------------------
        self.widget_8 = QtWidgets.QWidget(self.widget_3)
        self.widget_8.setGeometry(QtCore.QRect(int(10 * self.scale_x), int(420 * self.scale_y), 
                                            int(251 * self.scale_x), int(291 * self.scale_y)))
        self.widget_8.setStyleSheet("background-color: #1D1D1D;\n"
                                    "border: 1px solid #B9B9B9;")
        self.widget_8.setObjectName("widget_8")
        
        # Safe coordinates labels
        self.label_13 = QtWidgets.QLabel(self.widget_8)
        self.label_13.setGeometry(QtCore.QRect(int(25 * self.scale_x), int(40 * self.scale_y), 
                                int(241 * self.scale_x), int(41 * self.scale_y)))
        self.label_13.setStyleSheet("border: 0px;\n"
                                    "font-weight: 550;\n"
                                    "color: white;\n"
                                    f"font-size: {int(25 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_13.setObjectName("label_13")
        
        self.label_14 = QtWidgets.QLabel(self.widget_8)
        self.label_14.setGeometry(QtCore.QRect(int(52 * self.scale_x), int(3 * self.scale_y), 
                                int(181 * self.scale_x), int(41 * self.scale_y)))
        self.label_14.setStyleSheet("border: 0px;\n"
                                    "font-weight: bold;\n"
                                    "color: #A4A4A5;\n"
                                    f"font-size: {int(20 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_14.setObjectName("label_14")
        
        self.label_15 = QtWidgets.QLabel(self.widget_8)
        self.label_15.setGeometry(QtCore.QRect(int(25 * self.scale_x), int(80 * self.scale_y), 
                                int(241 * self.scale_x), int(41 * self.scale_y)))
        self.label_15.setStyleSheet("border: 0px;\n"
                                    "font-weight: 550;\n"
                                    "color: white;\n"
                                    f"font-size: {int(25 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_15.setObjectName("label_15")
        
        self.label_16 = QtWidgets.QLabel(self.widget_8)
        self.label_16.setGeometry(QtCore.QRect(int(25 * self.scale_x), int(120 * self.scale_y), 
                                int(241 * self.scale_x), int(41 * self.scale_y)))
        self.label_16.setStyleSheet("border: 0px;\n"
                                    "font-weight: 550;\n"
                                    "color: white;\n"
                                    f"font-size: {int(25 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_16.setObjectName("label_16")
        
        # --------------------------------------------------
        # Control Modes Widget (in right panel)
        # --------------------------------------------------
        self.widget_9 = QtWidgets.QWidget(self.widget_3)
        self.widget_9.setGeometry(QtCore.QRect(int(10 * self.scale_x), int(310 * self.scale_y), 
                                int(251 * self.scale_x), int(91 * self.scale_y)))
        self.widget_9.setStyleSheet("background-color: #1D1D1D;\n"
                                    "border: 1px solid #B9B9B9;")
        self.widget_9.setObjectName("widget_9")
        
        # Control mode labels and button
        self.label_17 = QtWidgets.QLabel(self.widget_9)
        self.label_17.setGeometry(QtCore.QRect(int(65 * self.scale_x), 0, int(181 * self.scale_x), int(41 * self.scale_y)))
        self.label_17.setStyleSheet("border: 0px;\n"
                                    "font-weight: bold;\n"
                                    "color: #A4A4A5;\n"
                                    f"font-size: {int(19 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_17.setObjectName("label_17")
        
        self.label_18 = QtWidgets.QLabel(self.widget_9)
        self.label_18.setGeometry(QtCore.QRect(int(40 * self.scale_x), int(40 * self.scale_y), 
                                int(141 * self.scale_x), int(41 * self.scale_y)))
        self.label_18.setStyleSheet("border: 0px;\n"
                                    "font-weight: 550;\n"
                                    "color: white;\n"
                                    f"font-size: {int(20 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_18.setObjectName("label_18")
        
        self.toolButton = QtWidgets.QToolButton(self.widget_9)
        self.toolButton.setGeometry(QtCore.QRect(int(180 * self.scale_x), int(47 * self.scale_y), 
                                    int(26 * self.scale_x), int(30 * self.scale_y)))
        self.toolButton.setStyleSheet("background-color: white;\n" "border-radius: 13px;")
        self.toolButton.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("assets/images/power.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton.setIcon(icon)
        self.toolButton.setObjectName("toolButton")
        
        # --------------------------------------------------
        # Altitude Widget (in right panel)
        # --------------------------------------------------
        self.widget_10 = QtWidgets.QWidget(self.widget_3)
        self.widget_10.setGeometry(QtCore.QRect(int(10 * self.scale_x), int(190 * self.scale_y), 
                                int(121 * self.scale_x), int(101 * self.scale_y)))
        self.widget_10.setStyleSheet("background-color: #1D1D1D;\n"
                                    "border: 1px solid #B9B9B9;")
        self.widget_10.setObjectName("widget_10")
        
        # Altitude display
        self.label_19 = QtWidgets.QLabel(self.widget_10)
        self.label_19.setGeometry(QtCore.QRect(int(35 * self.scale_x), int(40 * self.scale_y), 
                                int(81 * self.scale_x), int(51 * self.scale_y)))
        self.label_19.setStyleSheet("border: 0px;\n"
                                    "font-weight: 600;\n"
                                    "color: white;\n"
                                    f"font-size: {int(40 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_19.setObjectName("label_19")
        
        self.label_20 = QtWidgets.QLabel(self.widget_10)
        self.label_20.setGeometry(QtCore.QRect(int(30 * self.scale_x), 0, int(91 * self.scale_x), int(41 * self.scale_y)))
        self.label_20.setStyleSheet("border: 0px;\n"
                                    "font-weight: bold;\n"
                                    "color: #A4A4A5;\n"
                                    f"font-size: {int(18 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_20.setObjectName("label_20")
        
        # --------------------------------------------------
        # Battery Widget (in right panel)
        # --------------------------------------------------
        self.widget_11 = QtWidgets.QWidget(self.widget_3)
        self.widget_11.setGeometry(QtCore.QRect(int(140 * self.scale_x), int(190 * self.scale_y), 
                                int(121 * self.scale_x), int(101 * self.scale_y)))
        self.widget_11.setStyleSheet("background-color: #1D1D1D;\n"
                                    "border: 1px solid #B9B9B9;")
        self.widget_11.setObjectName("widget_11")
        
        # Battery display
        self.label_21 = QtWidgets.QLabel(self.widget_11)
        self.label_21.setGeometry(QtCore.QRect(int(29 * self.scale_x), int(37 * self.scale_y), 
                                            int(81 * self.scale_x), int(51 * self.scale_y)))
        self.label_21.setStyleSheet("border: 0px;\n"
                                    "font-weight: 600;\n"
                                    "color: white;\n"
                                    f"font-size: {int(36 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_21.setObjectName("label_21")
        
        self.label_22 = QtWidgets.QLabel(self.widget_11)
        self.label_22.setGeometry(QtCore.QRect(int(29 * self.scale_x), 0, int(91 * self.scale_x), int(41 * self.scale_y)))
        self.label_22.setStyleSheet("border: 0px;\n"
                                    "font-weight: bold;\n"
                                    "color: #A4A4A5;\n"
                                    f"font-size: {int(18 * min(self.scale_x, self.scale_y))}px;\n"
                                    "background-color: transparent;")
        self.label_22.setObjectName("label_22")
        
        # Drone image in right panel
        self.label = QtWidgets.QLabel(self.widget_3)
        self.label.setGeometry(QtCore.QRect(int(20 * self.scale_x), int(10 * self.scale_y), 
                            int(231 * self.scale_x), int(180 * self.scale_y)))
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap("assets/images/drone.png"))
        self.label.setScaledContents(True)
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        
        # ==============================================
        # Main Content Area with RTAB-Map Viewer
        # ==============================================
        # Main content area - responsive positioning and sizing
        main_content_width = screen_width - right_panel_width - int(62 * self.scale_x)
        main_content_height = screen_height - bottom_panel_height - int(80 * self.scale_y)
        
        self.widget_4 = QtWidgets.QWidget(self.widget)
        self.widget_4.setGeometry(QtCore.QRect(int(20 * self.scale_x), int(10 * self.scale_y), 
                                main_content_width, main_content_height))
        self.widget_4.setStyleSheet("border-radius: 10px;\n"
                                    "background-color: #101419;")
        self.widget_4.setObjectName("widget_4")
        
        # Create a layout for the main content area
        self.main_content_layout = QtWidgets.QVBoxLayout(self.widget_4)
        self.main_content_layout.setContentsMargins(0, 0, 0, 0)
        
        # Initialize RTAB-Map viewer
        self.rtabmap_viewer = RTABMapEmbed(self.widget_4)
        
        # Set the central widget
        MainWindow.setCentralWidget(self.widget)

        # Initialize translations and connections
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        # Start RTAB-Map capture when UI is ready
        self.rtabmap_viewer.start_capture()

        
    def setup_button_connections(self, MainWindow, node):
        # Initialize ROS publisher with the existing node
        self.autonomous_publisher = AutonomousModePublisher(node)
        
        # Connect button click event
        self.toolButton.clicked.connect(self.on_autonomous_button_clicked)
    
    def on_autonomous_button_clicked(self):
        # Create warning dialog
        msg_box = QtWidgets.QMessageBox()
        msg_box.setIcon(QtWidgets.QMessageBox.Warning)
        msg_box.setWindowTitle("Autonomous Mode Warning")
        msg_box.setText("Are you sure you want to enable autonomous mode?")
        msg_box.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        
        # Show dialog and wait for response
        response = msg_box.exec_()
        
        if response == QtWidgets.QMessageBox.Yes:
            # Publish True to autonomous_mode topic
            self.autonomous_publisher.publish_mode(True)
            
            # Update UI to show autonomous mode is active
            self.label_18.setText("Autonomous")
            self.toolButton.setStyleSheet("background-color: #5efc03;")
        else:
            # Publish False to autonomous_mode topic
            self.autonomous_publisher.publish_mode(False)
            
            # Update UI to show manual mode
            self.label_18.setText("Manual")
            self.toolButton.setStyleSheet("background-color: white;")

    def retranslateUi(self, MainWindow):
        """Set all the text labels and window title."""
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Drone Control Dashboard"))
        
        # Bottom panel labels
        self.label_2.setText(_translate("MainWindow", "Orientation"))
        self.label_7.setText(_translate("MainWindow", "X : 00.00"))
        self.label_8.setText(_translate("MainWindow", "Y : 00.00"))
        self.label_9.setText(_translate("MainWindow", "Z : 00.00"))
        self.label_3.setText(_translate("MainWindow", "Safety"))
        self.label_4.setText(_translate("MainWindow", "Battery  Low "))
        self.label_5.setText(_translate("MainWindow", "Lost  Link"))
        self.label_6.setText(_translate("MainWindow", "Velocity"))
        self.label_10.setText(_translate("MainWindow", "X : 00.00"))
        self.label_11.setText(_translate("MainWindow", "Y : 00.00"))
        self.label_12.setText(_translate("MainWindow", "Z : 00.00"))
        
        # Right panel labels
        self.label_13.setText(_translate("MainWindow", "1 : X:00.00  Y:00.00"))
        self.label_14.setText(_translate("MainWindow", "Safe Co-ordinates"))
        self.label_15.setText(_translate("MainWindow", "2 : X:00.00  Y:00.00"))
        self.label_16.setText(_translate("MainWindow", "3 : X:00.00  Y:00.00"))
        self.label_17.setText(_translate("MainWindow", "Control Modes"))
        self.label_18.setText(_translate("MainWindow", "Autonomous"))
        self.label_19.setText(_translate("MainWindow", "0.0"))
        self.label_20.setText(_translate("MainWindow", "Altitude"))
        self.label_21.setText(_translate("MainWindow", "00%"))
        self.label_22.setText(_translate("MainWindow", "Battery"))
       



if __name__ == "__main__":
    import sys
    from threading import Thread
    import rclpy
    from std_msgs.msg import Bool
    from new_sub import SensorSubscriber  # Your existing subscriber

    # Initialize ROS
    rclpy.init()
    node = rclpy.create_node('anav_ui_node')

    # Create Qt application
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)

    # Initialize publisher and subscriber
    ui.setup_button_connections(MainWindow, node)  # For autonomous mode
    sensor_subscriber = SensorSubscriber(ui)  # Your existing subscriber

    # ROS spin thread function
    def ros_spin():
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(sensor_subscriber)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            sensor_subscriber.destroy_node()

    # Start ROS thread
    ros_thread = Thread(target=ros_spin)
    ros_thread.daemon = True
    ros_thread.start()

    # Show main window
    MainWindow.show()

    # Cleanup on exit
    def shutdown():
        ui.rtabmap_viewer.stop_capture()
        rclpy.shutdown()
        ros_thread.join(timeout=1)

    app.aboutToQuit.connect(shutdown)
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        shutdown()

    import sys
    from threading import Thread
    import rclpy
    from std_msgs.msg import Bool
    from new_sub import SensorSubscriber  # Your existing subscriber

    # Initialize ROS
    rclpy.init()
    node = rclpy.create_node('anav_ui_node')

    # Create Qt application
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)

    # Initialize publisher and subscriber
    ui.setup_button_connections(MainWindow, node)  # For autonomous mode
    sensor_subscriber = SensorSubscriber(ui)  # Your existing subscriber

    # ROS spin thread function
    def ros_spin():
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(sensor_subscriber)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()

    # Start ROS thread
    ros_thread = Thread(target=ros_spin)
    ros_thread.daemon = True
    ros_thread.start()

    # Show main window
    MainWindow.show()

    # Cleanup on exit
    def shutdown():
        node.destroy_node()
        sensor_subscriber.destroy_node()
        rclpy.shutdown()

    app.aboutToQuit.connect(shutdown)
    sys.exit(app.exec_())