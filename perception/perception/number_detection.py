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
        self.yolo_model = YOLO("models/number_detection.pt")
        self.class_labels = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
        
    def camera_callback(self, msg):
        # see https://pyseek.com/2024/10/number-detection-system-using-python-yolov8/
        # convert between ROS Image messages and OpenCV images
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = imutils.resize(cv_image, width=640)

        # Perform object detection
        results = self.yolo_model(cv_image)

        # Loop through the detections and draw bounding boxes
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                w, h = x2 - x1, y2 - y1
                
                conf = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])

                if conf > 0.8:
                    cvzone.cornerRect(cv_image, (x1, y1, w, h), t=2)
                    cvzone.putTextRect(cv_image, f'{self.class_labels[cls]} {conf}', (x1, y1 - 10), scale=0.8, thickness=1, colorR=(255, 0, 0))

        # show image
        cv2.imshow("Number Detection", cv_image)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    number_detector = NumberDetector()
    rclpy.spin(number_detector)

    number_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
