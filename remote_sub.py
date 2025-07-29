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
        
        # Publisher and Subscriber
        self.autonomous_pub = self.create_publisher(Bool, '/autonomous_mode', 10)
        self.create_subscription(Bool, '/autonomous_mode', self.mode_callback, 10)
        
        # State & Process
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
        """Callback for incoming mode change"""
        self.set_mode(msg.data)

    def manage_test_script(self):
        """Start or stop the mat.py script based on mode"""
        if self.current_mode:
            self.start_test_script()
        else:
            self.stop_test_script()

    def start_test_script(self):
        """Launch mat.py if not already running"""
        if self.test_process is None:
            try:
                script_dir = os.path.dirname(os.path.abspath(__file__))
                test_script = os.path.join(script_dir, "mat.py")
                
                if os.path.exists(test_script):
                    self.get_logger().info("Launching mat.py...")

                    # Start subprocess with unbuffered output (-u)
                    self.test_process = subprocess.Popen(
                        ['python3', '-u', test_script],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        preexec_fn=os.setsid,
                        env=os.environ.copy()
                    )

                    # Monitor both stdout and stderr
                    threading.Thread(target=self.monitor_process, daemon=True).start()
                    threading.Thread(target=self.log_output, args=(self.test_process.stdout, 'stdout'), daemon=True).start()
                    threading.Thread(target=self.log_output, args=(self.test_process.stderr, 'stderr'), daemon=True).start()
                else:
                    self.get_logger().error(f"mat.py not found at: {test_script}")
                    self.set_mode(False)

            except Exception as e:
                self.get_logger().error(f"Failed to start mat.py: {str(e)}")
                self.test_process = None
                self.set_mode(False)

    def log_output(self, stream, label):
        """Log output from subprocess"""
        for line in iter(stream.readline, b''):
            self.get_logger().info(f"[{label}] {line.decode().strip()}")

    def stop_test_script(self):
        """Terminate the mat.py process if running"""
        if self.test_process is not None:
            try:
                self.get_logger().info("Stopping mat.py...")
                os.killpg(os.getpgid(self.test_process.pid), signal.SIGTERM)
                self.test_process.wait(timeout=2.0)

            except subprocess.TimeoutExpired:
                self.get_logger().warning("mat.py didn't terminate cleanly, forcing kill")
                os.killpg(os.getpgid(self.test_process.pid), signal.SIGKILL)
                self.test_process.wait()

            except ProcessLookupError:
                self.get_logger().warning("Process already terminated")

            except Exception as e:
                self.get_logger().error(f"Error stopping mat.py: {str(e)}")

            finally:
                self.test_process = None

    def monitor_process(self):
        """Wait for mat.py to exit and clean up"""
        if self.test_process:
            return_code = self.test_process.wait()
            self.get_logger().info(f"mat.py exited with code: {return_code}")

            with self.process_lock:
                self.test_process = None
                if return_code != 0 and self.current_mode:
                    self.get_logger().error("mat.py crashed! Disabling autonomous mode")
                    self.set_mode(False)

    def destroy_node(self):
        """Graceful shutdown"""
        self.stop_test_script()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = AutonomousController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
