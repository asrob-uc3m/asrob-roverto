#!/usr/bin/python3

import rclpy
import numpy as np
import cv2
import math
import cvzone
import imutils
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class PerceptionSpin(Node):
    global color, number

    def __init__(self):
        super().__init__('perception_spin_node')

        # Subscriber
        self.subscription = self.create_subscription(String,
                                                     '/perception/number_topic',
                                                     self.number_callback, 
                                                     1)
        self.subscription = self.create_subscription(String,
                                                     '/perception/box_color_topic',
                                                     self.color_callback, 
                                                     1)                                 
        self.subscription
        self.get_logger().info('PerceptionSpin subscribers are UP')

        # Publisher
        # self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_intuitive", 10)

        # Detection
        self.class_labels = {'uno': 1, 'dos': 2, 'tres': 3, 'cuatro': 4, 'cinco': 5, 'seis': 6, 'siete': 7, 'ocho': 8, 'nueve': 9}
        self.color, self.number = None, None

        # Spin
        self.spin_rate = 2.0 # angular rotation speed
        self.spin_time = 1.6 # hand tuned

    def number_callback(self, msg):
        number_name = msg.data
        self.get_logger().info(f'Got {number_name}')
        
        if number_name in self.class_labels.keys():
            self.number = self.class_labels[number_name]
            if self.color:
                self.spin()
        else:
            self.number = None

    def color_callback(self, msg):
        color_name = msg.data
        self.get_logger().info(f'Got {color_name}')

        if color_name != 'none':
            self.color = color_name
            if self.number:
                self.spin()
        else:
            self.color = None

    def spin(self):
        self.get_logger().info(f'Got {self.number} and {self.color}')
        # twist = Twist()
        if self.color == 'red':
            self.get_logger().info(f'Spinning {self.number} times to the right')
            # twist.angular.y = self.spin_rate
        elif self.color == 'blue':
            self.get_logger().info(f'Spinning {self.number} times to the left')
            # twist.angular.y = -self.spin_rate

        # spinning_time = 1.8 * math.pi * self.number / self.spin_rate
        # start_time = time.time()
        # current_time = time.time()
        # while current_time < start_time + spinning_time:
        #     self.cmd_vel_pub.publish(twist)
        #     current_time = time.time()

        # twist.angular.y = 0.0
        # self.cmd_vel_pub.publish(twist)
        # self.color = None
        # self.number = None


def main():
    rclpy.init()
    perception_spin = PerceptionSpin()
    rclpy.spin(perception_spin)

    perception_spin.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
