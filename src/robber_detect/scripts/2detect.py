#!/usr/bin/python3

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.node import Node

from cvfpscalc import CvFpsCalc
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import time
import copy

from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils

class RobberDetection(Node):

    def __init__(self):
        # Creates a node with name 'robber_detection_recognition' and make sure it is a
        # unique node (using anonymous=True).
        super().__init__('robber_detection_recognition')
        self.publisher=self.create_publisher(String,"/person_detected",10)
        self.subscription = self.create_subscription(Image,"/image_raw",self.callback,10)
        # Publisher which will publish to the topic 
        self.bridge = CvBridge()
        self.cv_fps_calc = CvFpsCalc(buffer_len=10)
   
        

    def callback(self, image_msg):
        """A callback function for the image subscriber

        Args:
            image_msg (sensor_msgs.msg): image message
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg)
            im=self.bridge.imgmsg_to_cv2(image_msg)
            fps = self.cv_fps_calc.get()
            cv.waitKey(10)

        
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
                file_name='/home/tolasing/main_ws/ai_ws/src/robber_detect/robber_detect/robber3.tflite', use_coral=False, num_threads=2)
            detection_options = processor.DetectionOptions(
                max_results=1, score_threshold=0.2)
            options = vision.ObjectDetectorOptions(
                base_options=base_options, detection_options=detection_options)
            detector = vision.ObjectDetector.create_from_options(options)
            counter += 1
            cv_image = cv.flip(cv_image, 1)

            # Convert the image from BGR to RGB as required by the TFLite model.
            rgb_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)

            # Create a TensorImage object from the RGB image.
            input_tensor = vision.TensorImage.create_from_array(rgb_image)

            # Run object detection estimation using the model.
            detection_result = detector.detect(input_tensor)

            # Draw keypoints and edges on input image
            rgb_image = utils.visualize(rgb_image, detection_result)
            #cv.imshow('robber detection',im)
            for detection in detection_result.detections:
                for category in detection.categories:
                    msg=String()
                    msg.data=category.category_name
                    self.publisher.publish(msg)
                    print("Category: {}".format(category.category_name))
                    
            # Calculate the FPS
            if counter % fps_avg_frame_count == 0:
                end_time = time.time()
                fps = fps_avg_frame_count / (end_time - start_time)
                start_time = time.time()

                # Show th FPS
                fps_text = 'FPS = {:.1f}'.format(fps)
                text_location = (left_margin, row_size)
                cv.putText(cv_image, fps_text, text_location, cv.FONT_HERSHEY_PLAIN,
                            font_size, text_color, font_thickness)
            cv.imshow('robber detection',rgb_image)      
              
        except CvBridgeError as error:
            print(error)

def main(args=None):
    rclpy.init(args=args)
    robber_detection = RobberDetection()
    rclpy.spin(robber_detection)
    # If we press control + C, the node will stop.
    robber_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
