#!/usr/bin/python3

import sys
import rclpy
import numpy as np
import cv2
import imutils
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msg.msg import Aruco


ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


class ArucoDetector(Node):
    def __init__(self):
        # Subscriber
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 
                                                     '/AstraProPlus/color/image_raw',
                                                     self.camera_callback, 
                                                     10)
        self.subscription

        # Publisher
        self.publisher_ = self.create_publisher(Aruco, 'aruco_topic', 1)

        # Distance
        self.camera_matrix = np.array([[1452.2952, 0, 923.2942], [0, 1441.3655, 528.0820], [0, 0, 1]])
        self.dist_coeffs = np.array((0.12, -0.3063, -0.003, -0.0071))
        self.marker_length = 0.268  # 30 cm

    def get_params(self):
        pass
        
    def camera_callback(self, msg):
        # https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
        # convert between ROS Image messages and OpenCV images
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = imutils.resize(cv_image, width=600)

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_5x5_50"])
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        corners, ids, rejected = detector.detectMarkers(cv_image)
        dist = []

        if len(corners) > 0:
            for (markerCorner, markerId) in zip(corners, ids):
                # top-left, top-right, bottom-right, and bottom-left order
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # compute and draw the center (x, y)-coordinates of the ArUco
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                # Distance
                rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.marker_length, self.camera_matrix, self.dist_coeffs)
                tvec = tvec[0][0]
                front_distance = tvec[2] * 0.366
                dist.append(front_distance)

            # Publish
            msg = Aruco()
            msg.distance = dist
            msg.id = ids
            self.publisher_.publish(msg)


def main():
    rclpy.init()
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
