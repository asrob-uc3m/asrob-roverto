#!/usr/bin/python3

import rclpy
import numpy as np
import cv2
import math
import cvzone
import imutils
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class LidarDetector(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # Subscriber
        self.subscription = self.create_subscription(LaserScan, 
                                                     '/scan',
                                                     self.lidar_callback,
                                                     qos_profile=qos_profile_sensor_data)
        self.subscription
        self.get_logger().info('LidarDetector subscriber is UP')
        
    def lidar_callback(self, msg):
        # values at 0 degree
        print(f'Distance left: {msg.ranges[0]}')

        # values at 90 degree
        print(f'Distance front: {msg.ranges[214]}')

        # values at 180 degree
        print(f'Distance right: {msg.ranges[429]}')


def main():
    rclpy.init()
    lidar_detector = LidarDetector()
    rclpy.spin(lidar_detector)

    lidar_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()