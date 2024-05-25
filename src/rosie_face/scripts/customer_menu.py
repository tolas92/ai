#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the necessary ROS2 message type
from tkinter import *
from threading import Thread
from geometry_msgs.msg import Twist
from rosie_face.face import *
from rosie_face.menu import *
from rclpy.action import ActionServer
from prototype.action import FoodMenu
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.timer import Timer
import time
from prototype.msg import Food
import math

class UiNode(Node):
    def __init__(self):
        super().__init__('UI_node')
        
        self.use_cmd_vel_for_face = True
        self.disable_cursor = False
        self.fullscreen = True
        self.counter=0
        
        # ROS2 publisher and subscribers
        self.subscription = self.create_subscription(Twist, '/diff_controller/cmd_vel_unstamped', self.cmd_vel_callback, 10)
        self.publisher=self.create_publisher(Food,'final_order',10)
        self.sub=self.create_subscription(String,'/ai_speaking',self.update_mouth,10)
   
        self.ttk = Tk()
        self.ttk.title("Bot UI")
        self.ttk.geometry("1024x600+0+0")
       #   self.update_timer = self.create_timer(1.0, self.update_image_callback)

        if self.fullscreen:
            #self.ttk.bind("<Escape>", self.end_fullscreen)
            self.ttk.attributes("-fullscreen",True)

        if self.disable_cursor:
            self.ttk.config(cursor="none")

        self.ttk.rowconfigure(0, weight=1)
        self.ttk.columnconfigure(0, weight=1)
 
        self.face_page = None
        self.button_page = None
        self.button=None
        self.face=None

        
        #self.build_button_page()
        self.build_face_page()
        
    
    def build_face_page(self):
        self.face_page = FacePlayerCars(self.ttk)
    
    def destroy_face_page(self):
        self.face_page.destroy()
        self.face_page = None

    def update_image(self):

        if self.button is True:
            self.destroy_face_page()
            self.build_button_page()
            self.button=None
        
        if self.face is True:
            self.destroy_button_page()
            self.build_face_page()
            self.face=None


        if self.face_page:
            self.face_page.update_image()

        if self.button_page:
            self.button_page.update_image()

        return
    def update_mouth(self,msg):
        self.destroy_face_page()
        self.build_button_page()
        
    def cmd_vel_callback(self, cmd_vel):
        
        if self.face_page and self.use_cmd_vel_for_face:
            self.face_page.update_values(cmd_vel.angular.z/1.0, abs(cmd_vel.linear.x/1.0))
            

        return
    
    def build_button_page(self):
        self.button_page = ButtonPage(self.ttk)

    def destroy_button_page(self):
        self.button_page.destroy()
        self.button_page = None 
    
def main(args=None):
    
    rclpy.init(args=args)
    ui_node = UiNode()
    executor=MultiThreadedExecutor()
    while rclpy.ok():

        rclpy.spin_once(ui_node,executor=executor,timeout_sec=1.0)
        time.sleep(0.01)
        ui_node.update_image()

    
    ui_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()