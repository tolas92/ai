#!/usr/bin/python3

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.node import Node
from hand_control.gesture_recognition import *
from cvfpscalc import CvFpsCalc
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import os
from ament_index_python.packages import get_package_share_directory
import time
import threading
import sys

class HandSignRecognition(Node):

    def __init__(self):
        super().__init__('hand_sign_recognition')
        self.subscription = self.create_subscription(Image, "/color/video/image", self.callback, 10)
        self.publisher_ = self.create_publisher(String, "/table_number", 10)
        self.gesture_detector = GestureRecognition("/home/tolasing/ai_ws/src/ros_hand_gesture_recognition/src/model/keypoint_classifier/keypoint_classifier_label.csv",
                                                "/home/tolasing/ai_ws/src/ros_hand_gesture_recognition/src/model/keypoint_classifier/keypoint_classifier.tflite")
        self.bridge = CvBridge()
        self.cv_fps_calc = CvFpsCalc(buffer_len=10)
        self.gesture = None
        self.lock = threading.Lock()
        self.publishing_thread_running = False


    def callback(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg)
            debug_image, self.gesture = self.gesture_detector.recognize(cv_image)

            if self.gesture == 'One'and not self.publishing_thread_running:
                # Run the table_publisher in a separate thread
                #threading.Thread(target=self.table_publisher, args=(self.gesture,)).start()
                # rclpy.shutdown()
                self.publishing_thread_running = True
                gesture_message = String()
                gesture_message.data = "one"
                self.publisher_.publish(gesture_message)
                cv.destroyAllWindows()

                rclpy.shutdown()
                return

                    
            fps = self.cv_fps_calc.get()
            debug_image = self.gesture_detector.draw_fps_info(debug_image, fps)
            cv.imshow('ROS Gesture Recognition', debug_image)
            cv.waitKey(10)  # wait for 10 milliseconds
        except CvBridgeError as error:
            print(error)

    def table_publisher(self, gesture):
        with self.lock:
            gesture_message = String()
            gesture_message.data = "one"
            self.publisher_.publish(gesture_message)
            cv.destroyAllWindows()
           # time.sleep(6000)
        with self.lock:
            self.publishing_thread_running = False

def main(args=None):
    rclpy.init(args=args)
    hand_sign = HandSignRecognition()
    start_time = time.time()
    while time.time() - start_time < 120:
        rclpy.spin_once(hand_sign, timeout_sec=1.0)  # Use a timeout to allow periodic checks

    hand_sign.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
