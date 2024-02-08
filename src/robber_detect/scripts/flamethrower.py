#! /usr/bin/env python3

import math # Math library
import time  # Time library

from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data # Handle quality of service for LaserScan data
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist # Velocity command
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState # Battery status
from sensor_msgs.msg import LaserScan # Handle LIDAR scans
from std_msgs.msg import Bool # Handle boolean values
from std_msgs.msg import Int32 # Handle integer values
from vision_msgs.msg import Detection3DArray
import robber_detect.voice_ as engine


# Flag to determine if the ArUco marker has been detected (True or False)
aruco_marker_detected = False

# Store the ArUco marker center offset (in pixels)
aruco_center_offset = 0
aruco_distance=0
distance_to_keep=1.5

# Keep track of obstacles in front of the robot in meters
obstacle_distance_front = 999999.9

class ConnectToChargingDockNavigator(Node):
    """
    Navigates and connects to the charging dock
    """      
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('connect_to_charging_dock_navigator')
    
      # Create a publisher
      # This node publishes the desired linear and angular velocity of the robot
      self.publisher_cmd_vel = self.create_publisher(
        Twist,
        '/diff_controller/cmd_vel_unstamped',
        10)  
      timer_period = 0.1
      self.timer = self.create_timer(timer_period, self.navigate_to_dock_staging_area)

      self.fire_command_pub=self.create_publisher(String,'/fire_command',10)

      # Declare linear and angular velocities
      self.linear_velocity = -0.005  # meters per second
      self.angular_velocity = 0.03 # radians per second
      self.docking_station_distance=1.8
      self.center_pid = PIDController(1.5,0.0,0.0)
      self.angular_pid=PIDController(0.7,0.0,0.0)
      self.linear_pid=PIDController(0.3,0.0,0.0)

      
      # Keep track of which goal we're headed towards
      self.goal_idx = 1
      
      # Declare obstacle tolerance 
      self.obstacle_tolerance = 0.22

      # Center offset tolerance in pixels
      self.center_offset_tolerance = 0
      self.orient_offset_tolerance=0.000
      self.diaganol_offset_tolerance=0.1
      
      # Undocking distance
      self.undocking_distance = 0.50
      self.perpendicular_line_x=0.0
        
    def navigate_to_dock_staging_area(self):
      """
      Navigate from somewhere in the environment to a staging area near
      the charging dock.
      """    
      self.get_logger().info('Low battery. Navigating to the charging dock...')
    
      self.connect_to_dock()
        
    def connect_to_dock(self): 
      """
      Go to the charging dock.
      """ 
        
      if (self.goal_idx == 1):
          self.navigate_to_aruco_marker()
          self.get_logger().info('Navigating to the ArUco marker...')
      else:
          # Stop the robot
          cmd_vel_msg = Twist()
          cmd_vel_msg.linear.x = 0.0
          cmd_vel_msg.angular.z = 0.0
          self.publisher_cmd_vel.publish(cmd_vel_msg)
          #self.get_logger().info('Arrived at charging dock. Robot is idle...')
        
          time.sleep(0.02)
        
          self.get_logger().info('CHARGING...')
          self.get_logger().info('Successfully connected to the charging dock!')
          cmd_vel_msg = Twist()
          cmd_vel_msg.linear.x = 0.0
          cmd_vel_msg.angular.z = 0.0
          self.publisher_cmd_vel.publish(cmd_vel_msg)
          
      # Reset the node
      #self.goal_idx = 0
    def search_for_aruco_marker(self):
      """
      Rotate around until the robot finds the charging dock
      """
      if aruco_marker_detected == False:
      
        # Create a velocity message
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = -self.angular_velocity           
      
        # Publish the velocity message  
        self.publisher_cmd_vel.publish(cmd_vel_msg) 
      else: 
        self.goal_idx = 1
        
    def navigate_to_aruco_marker(self):
      self.get_logger().info('calling adjust_heading')

      self.adjust_heading()

    def adjust_heading(self):
      """
      Adjust heading to keep the Aruco marker centerpoint centererd.

      """
      print(aruco_center_offset)
      cmd_vel_msg = Twist()

      
      if aruco_distance > distance_to_keep:
          self.perpendicular_line_x=aruco_distance*0.40
          center_correction = self.center_pid.calculate(aruco_center_offset - self.center_offset_tolerance)
          linear_correction=self.linear_pid.calculate(aruco_distance-distance_to_keep)
           

          cmd_vel_msg.angular.z=-center_correction
          cmd_vel_msg.linear.x=linear_correction
          self.publisher_cmd_vel.publish(cmd_vel_msg)


      if aruco_distance<distance_to_keep and aruco_distance>0.5:
         linear_correction=self.linear_pid.calculate(aruco_distance-distance_to_keep)
         cmd_vel_msg.linear.x=linear_correction
         self.publisher_cmd_vel.publish(cmd_vel_msg)
   
      else:
          print("hello")
              
class ArucoMarkerSubscriber(Node):
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('aruco_marker_subscriber')
    
      self.subscription_aruco_detected = self.create_subscription(
        String,
        '/person_detected', 
        self.person_callback,
        1)
      
      self.subscription_center_offset = self.create_subscription(
        Detection3DArray,
        '/oak/nn/spatial_detections', 
        self.get_center_offset,
        1)

      self.fire_publish = self.create_publisher(
        String,
        '/fire_command',
        10)  
      self.fire_count=0 
      self.warning_alert=engine
      self.warning_alert.load('')
        

    def person_callback(self, msg):
      global distance_to_keep
      global person_
      person_=msg.data
      if person_=="robber":
        self.fire_count=+1
        distance_to_keep=1
        if aruco_distance>distance_to_keep and self.fire_count>4:
          self.warning_alert.say("warning")
          self.fire_publish.publish("fire")
          self.fire_count=0
      else:
         distance_to_keep=1.5
         

        
    def get_center_offset(self, msg):
      """
      Update the ArUco marker center offset
      """
      global aruco_center_offset
      global aruco_orient_offset
      global aruco_distance

      for detection in msg.detections:
        if detection.results[0].hypothesis.class_id=='15':
          pose_=detection.results[0].pose.pose.position
          orien_=detection.results[0].pose.pose.orientation
          aruco_center_offset=round(pose_.x,1)
          aruco_orient_offset=round(orien_.z,1)
          aruco_distance=round(pose_.z,1)

"""
class FlamethrowerNode(Node):
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('Flamethrower_Node')
    
      self.subscription_aruco_detected = self.create_subscription(
        String,
        '/person_detected', 
        self.fire_callback,
        1)
            
      self.fire_publish = self.create_publisher(
        String,
        '/fire_command',
        10)  
      self.fire_count=0
"""        
            
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def calculate(self, error):
        # Proportional term
        p = self.kp * error

        # Integral term
        self.integral += error
        i = self.ki * self.integral

        # Derivative term
        derivative = error - self.prev_error
        d = self.kd * derivative

        # Update the previous error
        self.prev_error = error

        # Calculate the total correction
        correction = p + i + d

        return correction

        
def main(args=None):
  """
  Entry point for the program.
  """
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  try: 
  
    # Create the nodes
    connect_to_charging_dock_navigator = ConnectToChargingDockNavigator()
    aruco_marker_subscriber = ArucoMarkerSubscriber()
    
    # Set up mulithreading
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(connect_to_charging_dock_navigator)
    executor.add_node(aruco_marker_subscriber)
    
    try:
      # Spin the nodes to execute the callbacks
      executor.spin()
    finally:
      # Shutdown the nodes
      executor.shutdown()
      connect_to_charging_dock_navigator.destroy_node()
      aruco_marker_subscriber.destroy_node()

  finally:
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
  main()
