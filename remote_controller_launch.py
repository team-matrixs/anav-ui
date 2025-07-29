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
        self.launch_process = None
        self.process_lock = threading.Lock()
        
        self.get_logger().info("AutonomousController initialized")

    def set_mode(self, mode):
        with self.process_lock:
            if mode != self.current_mode:
                self.current_mode = mode
                msg = Bool()
                msg.data = self.current_mode
                self.autonomous_pub.publish(msg)
                self.get_logger().info(f"Autonomous mode set to: {self.current_mode}")
                self.manage_launch_process()

    def mode_callback(self, msg):
        self.set_mode(msg.data)

    def manage_launch_process(self):
        if self.current_mode:
            self.start_launch_file()
        else:
            self.stop_launch_file()

    def start_launch_file(self):
        if self.launch_process is None:
            try:
                # Adjust package and launch file name here
                package_name = 'test_pkg'  # ← Replace with your package name
                launch_file = 'yellow.launch.py'

                self.get_logger().info(f"Launching {launch_file} from package {package_name}...")

                self.launch_process = subprocess.Popen(
                    ['ros2', 'launch', package_name, launch_file],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,
                    env=os.environ.copy()
                )

                threading.Thread(target=self.monitor_process, daemon=True).start()
                threading.Thread(target=self.log_output, args=(self.launch_process.stdout, 'stdout'), daemon=True).start()
                threading.Thread(target=self.log_output, args=(self.launch_process.stderr, 'stderr'), daemon=True).start()

            except Exception as e:
                self.get_logger().error(f"Failed to start launch file: {str(e)}")
                self.launch_process = None
                self.set_mode(False)

    def stop_launch_file(self):
        if self.launch_process is not None:
            try:
                self.get_logger().info("Stopping launch file...")
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                self.launch_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warning("Launch process didn't terminate cleanly, forcing kill")
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGKILL)
                self.launch_process.wait()
            except ProcessLookupError:
                self.get_logger().warning("Launch process already terminated")
            except Exception as e:
                self.get_logger().error(f"Error stopping launch file: {str(e)}")
            finally:
                self.launch_process = None

    def log_output(self, stream, label):
        for line in iter(stream.readline, b''):
            self.get_logger().info(f"[{label}] {line.decode().strip()}")

    def monitor_process(self):
        if self.launch_process:
            return_code = self.launch_process.wait()
            self.get_logger().info(f"Launch file exited with code: {return_code}")

            with self.process_lock:
                self.launch_process = None
                if return_code != 0 and self.current_mode:
                    self.get_logger().error("Launch file crashed! Disabling autonomous mode")
                    self.set_mode(False)

    def destroy_node(self):
        self.stop_launch_file()
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
