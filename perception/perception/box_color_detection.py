#!/usr/bin/python3

import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class BoxColorDetector(Node):
    def __init__(self):
        super().__init__('box_color_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 
                                                     '/AstraProPlus/color/image_raw',
                                                     self.camera_callback, 
                                                     10)
        self.subscription
        
    def camera_callback(self, msg):
        # from https://agneya.medium.com/color-detection-using-python-and-opencv-8305c29d4a42
        # https://github.com/nimadorostkar/Color-Detector/blob/master/main.py
        # convert between ROS Image messages and OpenCV images
        imageFrame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        imageFrame = cv2.cvtColor(imageFrame,cv2.COLOR_RGB2BGR)
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # Set range for red color
        # https://docs.opencv.org/3.1.0/df/d9d/tutorial_py_colorspaces.html
        red = np.uint8([[[0, 0, 255]]])
        hsvRed = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
        lower_limit = hsvRed[0][0][0] - 10, 100, 100
        upper_limit = hsvRed[0][0][0] + 10, 255, 255
        red_mask = cv2.inRange(hsvFrame, np.array(lower_limit), np.array(upper_limit))

        # Set range for blue color
        blue = np.uint8([[[255, 0, 0]]]) # Here insert the BGR values which you want to convert to HSV
        hsvBlue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
        lower_limit = hsvBlue[0][0][0] - 10, 100, 100
        upper_limit = hsvBlue[0][0][0] + 10, 255, 255
        blue_mask = cv2.inRange(hsvFrame, np.array(lower_limit), np.array(upper_limit))

        # to detect only that particular color
        kernal = np.ones((5, 5), "uint8")

        # red color
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)

        # blue color
        blue_mask = cv2.dilate(blue_mask, kernal)
        res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        area_red = 0
        if contours:
            contour_red = max(contours, key = cv2.contourArea)
            area_red = cv2.contourArea(contour_red)

        # Creating contour to track blue color
        contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        area_blue = 0
        if contours:
            contour_blue = max(contours, key = cv2.contourArea)
            area_blue = cv2.contourArea(contour_blue)

        # Paint biggest area (none if equal)
        if area_red > area_blue:
            x, y, w, h = cv2.boundingRect(contour_red)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                    (x + w, y + h),
                                    (0, 0, 255), 2)

            cv2.putText(imageFrame, "Red Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))

        if area_blue > area_red:
            x, y, w, h = cv2.boundingRect(contour_blue)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                    (x + w, y + h),
                                    (255, 0, 0), 2)

            cv2.putText(imageFrame, "Blue Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))

        # final run
        cv2.imshow("Color Detection", imageFrame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    box_color_detector = BoxColorDetector()
    rclpy.spin(box_color_detector)

    box_color_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
