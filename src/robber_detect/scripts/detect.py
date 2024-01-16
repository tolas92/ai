#!/usr/bin/env python3
# Copyright 2021 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Main script to run the object detection routine."""
import argparse
import sys
import time

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.node import Node

from cvfpscalc import CvFpsCalc
from cv_bridge import CvBridge, CvBridgeError


import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils

class RobberDetect(Node):

  def __init__(self):
    super().__init__('robber_det_recognition')
    self.subscription = self.create_subscription(Image,"/image_raw",self.callback,10)
    self.bridge = CvBridge()
    self.cv_fps_calc = CvFpsCalc(buffer_len=10)

  def callback(self,image_msg):

    try:
        cv_image=self.bridge.imgmsg_to_cv2(image_msg)
        im=self.bridge.imgmsg_to_cv2(image_msg)
        cv2.imshow('robber detection',im)

        """
        # Variables to calculate FPS
          counter, fps = 0, 0
          start_time = time.time()

          # Visualization parameters
          row_size = 20  # pixels
          left_margin = 24  # pixels
          text_color = (0, 0, 255)  # red
          font_size = 1
          font_thickness = 1
          fps_avg_frame_count = 10

          # Initialize the object detection model
          base_options = core.BaseOptions(
              file_name='robber.tflite', use_coral=False, num_threads=4)
          detection_options = processor.DetectionOptions(
              max_results=3, score_threshold=0.3)
          options = vision.ObjectDetectorOptions(
              base_options=base_options, detection_options=detection_options)
          detector = vision.ObjectDetector.create_from_options(options)
          counter += 1
          cv_image = cv2.flip(cv_image, 1)

          # Convert the image from BGR to RGB as required by the TFLite model.
          rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

          # Create a TensorImage object from the RGB image.
          input_tensor = vision.TensorImage.create_from_array(rgb_image)

          # Run object detection estimation using the model.
          detection_result = detector.detect(input_tensor)

          # Draw keypoints and edges on input image
          cv_image = utils.visualize(cv_image, detection_result)
          #cv2.imshow('robber detection',im)

          # Calculate the FPS
          if counter % fps_avg_frame_count == 0:
              end_time = time.time()
              fps = fps_avg_frame_count / (end_time - start_time)
              start_time = time.time()

              # Show the FPS
              fps_text = 'FPS = {:.1f}'.format(fps)
              text_location = (left_margin, row_size)
              cv2.putText(cv_image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                          font_size, text_color, font_thickness)
          #cv2.imshow('robber detection',im)  
        """   
    except CvBridgeError as error:
      print(error)

def main(args=None):
    rclpy.init(args=args)
    robber_det=RobberDetect()
    rclpy.spin(robber_det)
    # If we press control + C, the node will stop.
    robber_det.destroy_node()
    rclpy.shutdown()

  




if __name__ == '__main__':
  main()
