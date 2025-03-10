#!/usr/bin/python3

import rclpy
import numpy as np
import cv2
import math
import cvzone
import imutils
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class NumberDetector(Node):
    def __init__(self):
        super().__init__('box_color_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 
                                                     '/AstraProPlus/color/image_raw',
                                                     self.camera_callback, 
                                                     10)
        self.subscription

        # Load YOLO model with custom weights
        self.yolo_model = YOLO("models/digits_profile-5.pt")
        self.class_labels = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
        
    def camera_callback(self, msg):
        # see https://pyseek.com/2024/10/number-detection-system-using-python-yolov8/
        # convert between ROS Image messages and OpenCV images
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = imutils.resize(cv_image, width=640)

        # Perform object detection
        results = self.yolo_model(cv_image, show=True, conf=0.8)

        # Print results
        if results:
            if results[0].boxes.conf[0] > 0.8:
                print(f'Found number {results[0].boxes.cls[0]} with confidence {results[0].boxes.conf[0]}')
            else:
                print('No numbers found with enough confidence')


def main():
    rclpy.init()
    number_detector = NumberDetector()
    rclpy.spin(number_detector)

    number_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
