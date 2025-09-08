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
from std_msgs.msg import String


class NumberDetector(Node):
    def __init__(self):
        super().__init__('box_color_node')
        self.bridge = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(Image, 
                                                     '/AstraProPlus/color/image_raw',
                                                     self.camera_callback, 
                                                     10)
        self.subscription
        self.get_logger().info('NumberDetector subscriber is UP')

        # Publisher
        self.publisher_ = self.create_publisher(String, 'number_topic', 1)
        self.get_logger().info('NumberDetector publisher is UP')

        # Load YOLO model with custom weights
        self.yolo_model = YOLO("/home/rover/osr_ws_uc3m/src/asrob-roverto/roverto/models/digits_color_bg-2.pt")
        self.names = self.yolo_model.names
        
    def camera_callback(self, msg):
        # see https://pyseek.com/2024/10/number-detection-system-using-python-yolov8/
        # convert between ROS Image messages and OpenCV images
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = imutils.resize(cv_image, width=640)

        # Perform object detection
        results = self.yolo_model(cv_image, show=False, conf=0.8)

        # Print results
        msg = String()
        msg.data = 'none'

        try:
            conf = results[0].boxes.conf[0]
        except: 
            conf = 0

        if conf > 0.8:
            msg.data = self.names[int(results[0].boxes.cls[0])]

        self.publisher_.publish(msg)


def main():
    rclpy.init()
    number_detector = NumberDetector()
    rclpy.spin(number_detector)

    number_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
