#! /usr/bin/env python3

import math # Math library
import time  # Time library
from rclpy.duration import Duration # Handles time for ROS 2
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data # Handle quality of service for LaserScan data
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist # Velocity command
from sensor_msgs.msg import BatteryState # Battery status
from sensor_msgs.msg import LaserScan # Handle LIDAR scans
from std_msgs.msg import Bool # Handle boolean values
from std_msgs.msg import Int32 # Handle integer values
from vision_msgs.msg import Detection3DArray


import rclpy
from rclpy.node import Node
import pygame
from rosie_school import spritesheet
import string
from std_msgs.msg import String

#action variable represents different expressions such as watching=1,talking=5,listening=3

action=1

class AnimationNode(Node):
    def __init__(self):
        super().__init__('animation_node')
        self.init_pygame()
        self.sub = self.create_subscription(String, "/eye_expression",self.call_,10)
        self.timer = self.create_timer(0.1,self.run_animation)

    def call_(self,msg):
        global action
        if msg.data=="watching":
            action=1
        elif msg.data=="listening":
            action=3
        elif msg.data=="talking":
            action=5


    def init_pygame(self):
        pygame.init()
        self.screen = pygame.display.set_mode((1000, 1000))
        pygame.display.set_caption('R.O.S.I.E')
        sprite_sheet_image = pygame.image.load('eyes.png').convert_alpha()
        self.sprite_sheet = spritesheet.SpriteSheet(sprite_sheet_image)
        self.bg = (50, 50, 50)
        self.animation_list = []
        animation_steps=[7,9,16,9,19,8]
        self.last_update = pygame.time.get_ticks()
        self.animation_cooldown = 50
        self.frame = 0
        self.step_counter = 0
        for animation in animation_steps:
            temp_img_list = []
            for _ in range(animation):
                temp_img_list.append(self.sprite_sheet.get_image(self.step_counter,900,900,1,(0, 0, 0)))
                self.step_counter += 1
            self.animation_list.append(temp_img_list)

    def run_animation(self):
      
        global action
        self.screen.fill(self.bg)
        current_time = pygame.time.get_ticks()
        if current_time - self.last_update >= self.animation_cooldown:
            self.frame += 1
            self.last_update = current_time
            if self.frame >= len(self.animation_list[action]):
                self.frame = 0
        self.screen.blit(self.animation_list[action][self.frame], (0, 0))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
        pygame.display.update()
    

def main(args=None):
  """
  Entry point for the program.
  """
  # Initialize the rclpy library
  rclpy.init(args=args)
  animation_node = AnimationNode()
  rclpy.spin(animation_node)
  rclpy.shutdown()

if __name__ == '__main__':
    main()
