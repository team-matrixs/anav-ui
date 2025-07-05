#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import os
import threading
import signal

class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')
        # Publisher (as in your original code)
        self.autonomous_pub = self.create_publisher(Bool, '/autonomous_mode', 10)
        
        # Subscriber for mode changes
        self.create_subscription(
            Bool,
            '/autonomous_mode',
            self.mode_callback,
            10
        )
        
        # Process management
        self.current_mode = False
        self.test_process = None
        self.process_lock = threading.Lock()
        
        self.get_logger().info("AutonomousController initialized")

    def set_mode(self, mode):
        """Set autonomous mode and publish to topic"""
        with self.process_lock:
            if mode != self.current_mode:
                self.current_mode = mode
                msg = Bool()
                msg.data = self.current_mode
                self.autonomous_pub.publish(msg)
                self.get_logger().info(f"Autonomous mode set to: {self.current_mode}")
                self.manage_test_script()

    def mode_callback(self, msg):
        """Handle incoming mode change messages"""
        self.set_mode(msg.data)

    def manage_test_script(self):
        """Start or stop test.py based on current mode"""
        if self.current_mode:
            self.start_test_script()
        else:
            self.stop_test_script()

    def start_test_script(self):
        """Start the test.py script if not already running"""
        if self.test_process is None:
            try:
                # Get path to test.py in the same directory as this script
                script_dir = os.path.dirname(os.path.abspath(__file__))
                test_script = os.path.join(script_dir, "mat.py")
                
                if os.path.exists(test_script):
                    self.get_logger().info("Launching test.py...")
                    
                    # Start the process with clean environment
                    self.test_process = subprocess.Popen(
                        ['python3', test_script],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        preexec_fn=os.setsid,  # For proper process group management
                        env=os.environ.copy()
                    )
                    
                    # Start monitoring thread
                    threading.Thread(
                        target=self.monitor_process,
                        daemon=True
                    ).start()
                else:
                    self.get_logger().error(f"test.py not found at: {test_script}")
                    self.set_mode(False)  # Revert mode if script not found
            except Exception as e:
                self.get_logger().error(f"Failed to start test.py: {str(e)}")
                self.test_process = None
                self.set_mode(False)  # Revert mode on failure

    def stop_test_script(self):
        """Stop the test.py script if running"""
        if self.test_process is not None:
            try:
                self.get_logger().info("Stopping test.py...")
                
                # Send SIGTERM to entire process group
                os.killpg(os.getpgid(self.test_process.pid), signal.SIGTERM)
                
                # Wait for clean termination
                self.test_process.wait(timeout=2.0)
                
            except subprocess.TimeoutExpired:
                self.get_logger().warning("test.py didn't terminate cleanly, forcing kill")
                os.killpg(os.getpgid(self.test_process.pid), signal.SIGKILL)
                self.test_process.wait()
                
            except ProcessLookupError:
                self.get_logger().warning("Process already terminated")
                
            except Exception as e:
                self.get_logger().error(f"Error stopping test.py: {str(e)}")
                
            finally:
                self.test_process = None

    def monitor_process(self):
        """Monitor the test.py process and handle termination"""
        if self.test_process:
            return_code = self.test_process.wait()
            self.get_logger().info(f"test.py exited with code: {return_code}")
            
            with self.process_lock:
                self.test_process = None
                
                # If we were in autonomous mode when it crashed, disable mode
                if return_code != 0 and self.current_mode:
                    self.get_logger().error("test.py crashed! Disabling autonomous mode")
                    self.set_mode(False)

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.stop_test_script()
        super().destroy_node()


# Example standalone test code
def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the controller
    controller = AutonomousController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()