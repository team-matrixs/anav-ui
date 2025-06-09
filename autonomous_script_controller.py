#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import subprocess
import os

class AutonomousScriptController:
    def __init__(self):
        self.script_process = None
        # Path to your Python script (update this)
        self.script_path = os.path.expanduser("./experiment.py")
        
        # Verify the script exists during initialization
        if not os.path.isfile(self.script_path):
            rospy.logerr(f"Python script not found at: {self.script_path}")
            rospy.signal_shutdown("Script not found")
            return
            
        rospy.init_node('autonomous_script_controller')
        rospy.Subscriber('/autonomous_mode', Bool, self.mode_callback)
        rospy.loginfo("Autonomous script controller ready...")
        
    def mode_callback(self, msg):
        """Handle autonomous mode changes"""
        if msg.data:
            self.start_script()
        else:
            self.stop_script()
            
    def start_script(self):
        """Start the Python script"""
        if self.script_process is None:
            try:
                rospy.loginfo(f"Starting Python script: {self.script_path}")
                self.script_process = subprocess.Popen(
                    ["python3", self.script_path],  # Note: Using python3 explicitly
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
            except Exception as e:
                rospy.logerr(f"Failed to start Python script: {str(e)}")
                
    def stop_script(self):
        """Stop the Python script"""
        if self.script_process is not None:
            rospy.loginfo("Stopping Python script...")
            self.script_process.terminate()
            try:
                self.script_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.script_process.kill()
            self.script_process = None

if __name__ == "__main__":
    controller = AutonomousScriptController()
    rospy.spin()