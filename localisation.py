#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu  # Add this import
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.srv import CommandBool, CommandTOL
from mavros_msgs.srv import SetMode
from test_pkg.srv import SafeSpots
import math
from std_msgs.msg import String, Bool, Float32MultiArray
import subprocess
import json

def read_setpoints_from_file(filename="/home/mjet/test_ws/setpoints.json"):
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            setpoints = data["setpoints"]
            return [
                tuple(setpoints["s1"]),
                tuple(setpoints["s2"]),
                tuple(setpoints["s3"]),
                tuple(setpoints["s4"]),
                tuple(setpoints["s5"])
            ]
        except FileNotFoundError:
            print("❌ File not found.")
            return []
        except KeyError as e:
            print(f"❌ Missing key: {e}")
            return []
        except json.JSONDecodeError:
            print("❌ JSON decoding error.")
            return []

def read_center_from_file(filename="/home/mjet/test_ws/setpoints.json"):
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            center = data["center"]
            return [
                center["center_x"],
                center["center_y"]
               
            ]
        except FileNotFoundError:
            print("❌ File not found.")
            return []
        except KeyError as e:
            print(f"❌ Missing key: {e}")
            return []
        except json.JSONDecodeError:
            print("❌ JSON decoding error.")
            return []



class Drone(Node):
    def __init__(self):
        super().__init__('drone_node')
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        print("nrubenrungunebetrnhu")
        
        # Subscriptions
        self.sub_pose = self.create_subscription(PoseStamped,'/mavros/local_position/pose',self.listener,qos_best_effort)
        self.sub_state = self.create_subscription(State,'/mavros/state',self.state_callback,qos_reliable)

        self.sub_imu = self.create_subscription(Imu,'/mavros/imu/data',self.imu_callback,qos_best_effort)

        self.publisher_ = self.create_publisher(Twist,'/mavros/setpoint_velocity/cmd_vel_unstamped',qos_reliable)

        self.sub_waypoint = self.create_subscription(String,'/waypoint',self.waypoint_callback,qos_profile=qos_reliable)
        
        self.pub_bool = self.create_publisher(Bool, 'my_bool_topic', 10)
        
        self.sub_list = self.create_subscription(Float32MultiArray,'float_list_topic',self.list_callback,10)

        self.pub_coordinates = self.create_publisher(String, '/text_topic', 10)
        self.coordinate_counter = 1  # Counter for coordinate numbering

        # Service clients
        self.arm_cli     = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_cli    = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.safe_spots_cli  = self.create_client(SafeSpots, '/get_safe_spots')

        # State and pose
        self.setpoint_msg = Twist()
        self.bool_msg = Bool()
        self.bool_msg.data = False
        self.origin_flag = True
        self.state       = 'IDLE'
        self.mode        = None
        self.armed       = False

        self.proportionality_constant = 0.2
        
        self.pose_x = self.pose_y = self.pose_z = self.angular_z = 0.0
        self.origin_x = self.origin_y = self.origin_z = 0.0
        self.orientation_x = self.orientation_y = self.orientation_z = self.orientation_w = 0.0
        self.timer = self.rel_z = self.rel_x = self.rel_y = 0.0
        self.para_x = self.para_y = self.move_timer = self.yaw_degrees = 0.0
        self.list_flag = self.drift_correction_flag = None
        self.drift_x = self.drift_y = 0.0
        self.shifted_x = self.shifted_y = None
        self.proceed_flag = True
        self.vel_x =  self.vel_y = self.vel_z = self.angular_z = 0.0
        self.rescan_block_flag = True

        self.yaw_euler = self.desired_angular_z = 0.0
        self.current_list = self.current_waypoint = 0
        self.takeoff_flag = self.mode_flag = self.arm_flag = self.safespot_flag = self.extract_parameters_flag = self.first_coordinate =  self.origin_flag_yaw = False
        self.start_time = self.start_time_takeoff = self.feedback = None
        self.arm_flag_inner = True
        self.temp_list = []
        self.safespot_list = self.transformed_safe_spots = []
        self.cx , self.cy = read_center_from_file()

        setpts = read_setpoints_from_file()
        
        #self.base_waypoints1 = [(1.0,0.0,2.2),(1.0,1.0,2.2),(1.0,-1.0,2.2)]
        self.base_waypoints1 = setpts
        self.waypoint_list = [self.base_waypoints1]
        
        self.waypoint = self.waypoint_list[self.current_list]  

    def publish_current_coordinates(self):
        """Publish the current coordinates of the drone as formatted string"""
        coordinate_msg = String()
        coordinate_msg.data = f"{self.coordinate_counter} : X:{(self.rel_x + (self.cx)):05.2f} Y:{(self.rel_y + (self.cy)):05.2f}"
        self.pub_coordinates.publish(coordinate_msg)
        self.get_logger().info(f'📍 Published coordinate string: {coordinate_msg.data}')
        self.coordinate_counter += 1

    def list_callback(self,msg : Float32MultiArray):
        print('in list callback')
        self.list_flag, self.drift_x, self.drift_y = msg.data
        if self.drift_x > 2.0 or self.drift_y > 2.0:
            self.drift_x = self.drift_y = 0.0
        if self.list_flag == 1.0:
            print("drift correction recieved")
            temp_list = [self.rotate_2d_list(p, 270) for p in [(self.drift_x,self.drift_y)]]
            self.drift_x , self.drift_y  = temp_list[0] 
            self.get_logger().warn(f'drift recieved is {self.drift_x} and {self.drift_y}')
            self.drift_correction_flag = True                                                             #need to terminate flag
        
    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to euler angles (roll, pitch, yaw)
        Returns angles in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def imu_callback(self, msg: Imu):
        """
        Callback for IMU data - extracts and prints yaw in euler form
        """
        # Extract quaternion from IMU
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Convert to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        
        # Store yaw for use in other functions
        self.yaw_euler = yaw
        
        # Convert to degrees for easier reading
        self.yaw_degrees = math.degrees(yaw)
        if self.origin_flag_yaw == False:
            self.desired_angular_z = self.yaw_degrees
            self.origin_flag_yaw = True
        
        # Print yaw continuously
        self.get_logger().info(f'IMU Yaw: {self.yaw_degrees:.2f}° ({yaw:.4f} rad)')
    
    def waypoint_callback(self, msg: String):
        self.feedback = msg.data
        print(self.feedback)

    def listener(self, msg: PoseStamped):
        # Update current pose
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_z = msg.pose.position.z
        self.orientation_w = msg.pose.orientation.w
        self.orientation_z = msg.pose.orientation.z
        self.orientation_y = msg.pose.orientation.y
        self.orientation_x = msg.pose.orientation.x

        if self.origin_flag:
            self.origin_x = self.pose_x
            self.origin_y = self.pose_y
            self.origin_z = self.pose_z
            self.origin_flag = False
            self.get_logger().info('Origin set for relative positioning')

        # Log relative position
        self.rel_x = self.pose_x - self.origin_x
        self.rel_y = self.pose_y - self.origin_y
        self.rel_z = self.pose_z - self.origin_z
        '''self.rel_x = self.pose_x
        self.rel_y = self.pose_y
        self.rel_z = self.pose_z'''
        self.get_logger().info(
            f'[listener] X: {self.rel_x:.4f}, Y: {self.rel_y:.4f}, Z: {self.pose_z:.4f}'
        )

    def state_callback(self, msg: State):
        self.armed = msg.armed
        self.mode  = msg.mode
        self.get_logger().info(f'Received state: armed={self.armed}, mode={self.mode}')

    def rotate_2d_list(self,point: list, angle_deg: float) -> list:
        """
        Rotate a 2D point represented as a list [x, y] by angle_deg around the origin.
        """
        θ = math.radians(angle_deg)
        cosθ = math.cos(θ)
        sinθ = math.sin(θ)
        x, y = point
        x_new = cosθ * x - sinθ * y
        x_new += self.para_x
        x_new = -1.0*x_new
        y_new = sinθ * x + cosθ * y
        y_new += self.para_y
        y_new = -1.0*y_new
        print("rotated x & y:",x_new,y_new)
        return [x_new, y_new]
    
    def expand_safe_spots(self):
        for i in self.transformed_safe_spots:
            x,y = i
            if self.transformed_safe_spots.index(i) == 0:
                self.waypoint_list.append(([(self.base_waypoints1[len(self.base_waypoints1)-1]),(x,y,2.0)]))
            else:
                self.waypoint_list.append(([(x , y , 2.2)]))
                #self.waypoint_list.append(([(previous_x,previous_y,2.0),(x, y , 2.0)]))
            previous_x, previous_y = x, y    
            self.get_logger().info(f'Expanded waypoint with safe spot: {x:.2f}, {y:.2f}')
            self.get_logger().info(f'the current waypoint list is: {self.waypoint_list}')

    def request_safe_spots(self):
        print('[request] Calling safe spots service')
        req = SafeSpots.Request()
        future = self.safe_spots_cli.call_async(req)
        future.add_done_callback(self.safe_spots_response_callback)
        print('Service request sent')

    def safe_spots_response_callback(self, future):                                  #retrieves saespots and appends it in list self.safespot_list                                                                                    
            res = future.result()                                                    ##will add transformation,translation,wapoint list creation in same loop,done
            if res.success and res.x_coords:
                for z, y in zip(res.x_coords, res.y_coords):
                    print(f"{z:.2f}, {y:.2f}")
                    #transformation and translation
                    self.safespot_list.append([z,y])
                self.transformed_safe_spots = [self.rotate_2d_list(p, 270) for p in self.safespot_list]
            self.expand_safe_spots()
            self.waypoint_list.append([(-self.cx,-self.cy,2.0)])
            self.safespot_flag =  True
            self.state = 'LAND'

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.state = 'TAKEOFF'
        self.start_time = time.time()

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        future = self.disarm_cli.call_async(req)
        future.add_done_callback(self.disarm_response_callback)

    def disarm_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info('Drone disarmed successfully')
                self.state = 'WAIT'
            else:
                self.get_logger().warn('Failed to disarm the drone')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def takeoff(self, height: float = 0.0):
        req = CommandTOL.Request()
        req.altitude  = height
        req.latitude  = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw       = 0.0
        future = self.takeoff_cli.call_async(req)
        future.add_done_callback(self.takeoff_response_callback)

    def takeoff_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info('Takeoff succeeded')
                self.state = 'MOVE'
                self.start_time_takeoff = time.time()
            else:
                self.get_logger().warn('Takeoff failed')
                self.state = 'MOVE'
                self.start_time_takeoff = time.time()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def land(self):
        req = CommandTOL.Request()
        req.altitude  = 0.0
        req.latitude  = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw       = 0.0
        future = self.land_cli.call_async(req)
        future.add_done_callback(self.land_response_callback)

    def land_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info('Landing succeeded!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                self.state = 'IDLE'
                self.timer = time.time()
                #subprocess.Popen(['python3', '/home/mjet/test_ws/src/test_pkg/scripts/land_mavros_reset.py'])
                self.mode_flag = True
                self.takeoff_flag = False
                self.arm_flag = True
                self.rescan_block_flag = True
                self.get_logger().info('Setting mode to GUIDED...')

            else:
                self.get_logger().warn('Landing failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def set_mode(self, custom_mode: str):
        if not self.set_mode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetMode service not available')
            return

        req = SetMode.Request()
        req.custom_mode = custom_mode
        req.base_mode = 0  # Let MAVROS handle base_mode
        future = self.set_mode_cli.call_async(req)
        future.add_done_callback(self.set_mode_response_callback)

    def set_mode_response_callback(self, future):
        try:
            res = future.result()
            if res.mode_sent:
                self.get_logger().info(f'Mode set to GUIDED successfully')
            else:
                self.get_logger().warn(f'Failed to set mode to GUIDED')
        except Exception as e:
            self.get_logger().error(f'SetMode service call failed: {e}')

    def extract_parameters(self):                #if accuracy drops i will need to add theta
        self.para_x = self.rel_x                 #add flag to only run once
        self.para_y = self.rel_y

    def move(self,desired_x,desired_y,desired_z):
        velocity_x = max(-0.3, min(self.proportionality_constant * (desired_x - self.rel_x),0.3))
        velocity_y = max(-0.3, min(self.proportionality_constant * (desired_y - self.rel_y),0.3))
        velocity_z = max(-0.05, min(self.proportionality_constant * (desired_z - self.rel_z),0.05))
        angular_z = max(-0.05, min(0.05*(self.desired_angular_z - self.yaw_degrees),0.05))
        return round(velocity_x,3), round(velocity_y,3), round(velocity_z,3), round(angular_z,3)
    
    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=5.0)
            if self.feedback is not None and self.extract_parameters_flag ==False:
                self.extract_parameters()                 #retrives current x,y
                self.extract_parameters_flag = True

            if self.arm_flag and time.time() - self.timer > 13.0:
                self.set_mode('LOITER')
                self.get_logger().info('Setting mode to LOITER...')
            if self.arm_flag and time.time() - self.timer > 17.0:
                self.arm()
                self.get_logger().info('Arming drone...')
                self.arm_flag = False

            if self.mode_flag and time.time() - self.timer > 22.0:
                self.set_mode('GUIDED')
                self.get_logger().info('Setting mode to GUIDED...')
                self.mode_flag = False

            if self.mode == 'GUIDED':
                if self.current_list == 1 and self.current_waypoint == 0:
                    self.state = 'MOVE'

                if self.state == 'IDLE':
                    if self.arm_flag_inner:
                        #self.arm()
                        self.arm_flag_inner = False
                        self.state = 'TAKEOFF'
                        self.start_time = time.time()
                    #self.state = 'WAIT'

                elif self.state == 'TAKEOFF':
                    if self.start_time and time.time() - self.start_time > 3.0:
                        self.takeoff(2.2)

                elif self.state == 'MOVE':
                    self.move_timer = time.time()
                    if time.time() - self.timer > 5.0:
                        self.move_timer = 0.0
                        if self.first_coordinate:
                                dx = abs(x - (self.pose_x - self.origin_x))
                                dy = abs(y - (self.pose_y - self.origin_y))
                                dz = abs(z - (self.pose_z - self.origin_z))
                                if dx < 0.3 and dy < 0.3:
                                                                       
                                    self.get_logger().info(f'Waypoint {self.current_waypoint} reached')
                                    if self.current_waypoint == ((len(self.waypoint))-1) and self.safespot_flag == False:
                                        print("requesting safespot")
                                        self.request_safe_spots()

                                       
                                    elif self.current_waypoint == ((len(self.waypoint))-1) and self.safespot_flag == True:
                                        self.state = 'LAND'
                                    else:
                                        print("entered loop")
                                    self.current_waypoint += 1
                        if self.start_time_takeoff and self.feedback is not None and time.time()-self.start_time_takeoff>5.0:
                            if self.current_waypoint < len(self.waypoint):
                                x, y, z = self.waypoint[self.current_waypoint]
                                self.vel_x, self.vel_y,self.vel_z,self.angular_z = self.move(x, y, z)
                                #if self.current_waypoint != len(self.waypoint) - 1:
                                #    self.vel_z = 0.0
                                self.setpoint_msg.linear.x = self.vel_x
                                self.setpoint_msg.linear.y = self.vel_y
                                self.setpoint_msg.linear.z = 0.0
                                self.setpoint_msg.angular.z = self.angular_z
                                self.publisher_.publish(self.setpoint_msg)
                                self.vel_x = self.vel_y = self.angular_z = 0.0
                                print('setpoint along x and y',x,y)
                                self.first_coordinate = True
                        
                        
                elif self.state == 'LAND':
                    self.get_logger().info('[run] Initiating LAND')
                    
                    if self.current_list != 0:
                        if self.rescan_block_flag == True:
                            self.bool_msg.data = True
                            self.get_logger().warn('sent request for rescan')
                            self.rescan_block_flag = False
                        self.proceed_flag = None
                        self.vel_x = self.vel_y = self.vel_z = self.angular_z = 0.0                                      
                        if self.drift_correction_flag == True:
                            print('in drift corrcetion loop')
                            if self.shifted_x == None:
                                self.shifted_x = self.rel_x                         
                                self.shifted_y = self.rel_y
                            self.vel_x, self.vel_y,self.vel_z,self.angular_z = self.move(self.drift_x + self.shifted_x,self.drift_y + self.shifted_y, 2.0)
                            self.get_logger().info(f'moving towards {self.shifted_x + self.drift_x} and {self.shifted_y + self.drift_y}')
                        
                            if abs((self.drift_x + self.shifted_x) - self.rel_x) < 0.2 and abs((self.drift_y + self.shifted_y) - self.rel_y) < 0.2:
                                self.get_logger().warn(f'nullified drift ...')
                                self.publish_current_coordinates()
                                self.land()
                                self.drift_correction_flag = False
                                self.shifted_x = self.shifted_y = None
                                self.proceed_flag = True
                        self.setpoint_msg.linear.x = self.vel_x
                        self.setpoint_msg.linear.y = self.vel_y
                        self.setpoint_msg.linear.z = 0.0
                        self.setpoint_msg.angular.z = self.angular_z
                        self.publisher_.publish(self.setpoint_msg)
                        self.vel_x = self.vel_y = self.angular_z = 0.0
                                      
                    if self.current_list < len(self.waypoint_list) and self.proceed_flag == True:                 
                        self.current_list += 1
                        if self.current_list == len(self.waypoint_list):
                            self.state = 'WAIT'
                        self.waypoint = self.waypoint_list[self.current_list]
                        self.current_waypoint = 0
                        self.first_coordinate = False
                        self.start_time = None
                        
                        if self.current_list == 1:
                            self.state = 'MOVE'
                        else:
                            self.start_time_takeoff = None
                            self.arm_flag = True
                            self.mode_flag = True
                            self.proceed_flag = None
                            self.drift_x = self.drift_y = None
                            self.timer = time.time()
                        self.get_logger().info(f'Switching to next waypoint list: {self.waypoint}')
                    
            
            
            self.pub_bool.publish(self.bool_msg)
            self.bool_msg.data = False
            self.bool_value = False
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    drone = Drone()
    drone.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()