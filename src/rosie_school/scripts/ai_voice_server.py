#! /usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from prototype.action import AiCamera
from ollama import chat
import ollama
import tkinter as tk
from std_msgs.msg import String
import threading
from rosie_school import voice_ as engine
from rosie_school import *
import pygame
import string
from std_msgs.msg import String



action=1

class CoActionServer(Node):

    def __init__(self):
        super().__init__('ai_voice_server')
        self._action_server = ActionServer(
            self,
            AiCamera,
            '/AiCamera',
            self.execute_callback)
        self.voice_ai=engine
        self.voice_ai.load('')
        self.eye_expression_publisher=self.create_publisher(String,"/eye_expression",10)

      

    def execute_callback(self, goal_handle):
        global action
        self.get_logger().info('Executing goal...')
        self.publish_eye_expression("talking")
        goal=goal_handle.request.user_request
        self.msg_list.insert(tk.END, f"You: {goal}\n")
        feedback_msg=AiCamera.Feedback()
    
        stream = ollama.chat(
        model='orca-mini:3b',
        messages=[{'role': 'user', 'content': goal}],
        stream=True,
    )

        self.msg_list.insert(tk.END, f"Assitant:\n")
        for chunk in stream:  # Loop for 10 seconds
        # feedback_msg.distance_left = float(i)  # Update x-coordinate 
            #self.get_logger().info(f'Feedback: x={i},')
            self.publish_eye_expression("talking")
            feedback_msg.ai_response=chunk['message']['content']
            self.voice_ai.say(chunk['message']['content'])
            goal_handle.publish_feedback(feedback_msg)
            
            self.msg_list.insert(tk.END,chunk['message']['content'])   

        self.publish_eye_expression("watching")
        self.msg_list.insert(tk.END,'\n')
        goal_handle.succeed()
        result = AiCamera.Result()
        result.ai_end="."
        return result
        #rclpy.shutdown()

    def send_message(self):
        msg = String()
        msg.data = self.entry.get()
        self.publisher_.publish(msg)
        self.entry.delete(0, tk.END)

    def publish_eye_expression(self, expression):
        msg = String()
        msg.data = expression
        self.eye_expression_publisher.publish(msg)

def run_ros_node(node):
    rclpy.spin(node)

def on_close(node, root):
    node.destroy_node()
    rclpy.shutdown()
    root.destroy()


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = CoActionServer()
      # Set up the GUI in the main thread
    root = tk.Tk()
    root.title("ROS2 Chat")
    fibonacci_action_server.msg_list = tk.Text(root, height=15, width=50)
    fibonacci_action_server.msg_list.pack(padx=5, pady=5)
    fibonacci_action_server.entry = tk.Entry(root)
    fibonacci_action_server.entry.pack(padx=5, pady=5)
    fibonacci_action_server.button = tk.Button(root, text="Send", command=fibonacci_action_server.send_message)
    fibonacci_action_server.button.pack(padx=5, pady=5)
    root.protocol("WM_DELETE_WINDOW", lambda: on_close(fibonacci_action_server, root))

    # Run the ROS node in a separate thread
    ros_thread = threading.Thread(target=run_ros_node, args=(fibonacci_action_server,))
    ros_thread.start()

    # Start the Tkinter event loop in the main thread
    root.mainloop()


if __name__ == '__main__':
    main()