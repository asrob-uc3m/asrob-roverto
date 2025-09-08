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
from time import sleep
import sys
from os import path
# need to add the roboclaw.py file in the path
sys.path.append(path.join(path.expanduser('~'), 'osr_ws_uc3m/src/osr-rover-code/ROS/osr_control/osr_control'))
from roboclaw import Roboclaw
from adafruit_servokit import ServoKit


BAUD_RATE = 115200


class PerceptionSpin(Node):
    def __init__(self):
        super().__init__('perception_spin_node')

        # Subscriber
        self.subscription = self.create_subscription(String,
                                                     '/roverto/number_topic',
                                                     self.number_callback, 
                                                     1)
        self.subscription = self.create_subscription(String,
                                                     '/roverto/box_color_topic',
                                                     self.color_callback, 
                                                     1)                                 
        self.subscription
        self.get_logger().info('PerceptionSpin subscribers are UP')

        # Detection
        self.class_labels = {'uno': 1, 'dos': 2, 'tres': 3, 'cuatro': 4, 'cinco': 5, 'seis': 6, 'siete': 7, 'ocho': 8, 'nueve': 9}
        self.color, self.number = None, None

        # Spin
        self.lap_time = 7

    def test_connection(self, address):
        roboclaw0 = Roboclaw("/dev/serial0", BAUD_RATE)
        roboclaw1 = Roboclaw("/dev/serial1", BAUD_RATE)
        connected0 = roboclaw0.Open() == 1
        connected1 = roboclaw1.Open() == 1
        if connected0:
            self.get_logger().info("Connected to /dev/serial0.")
            self.get_logger().info(f"version: {roboclaw0.ReadVersion(address)}")
            self.get_logger().info(f"encoders: {roboclaw0.ReadEncM1(address)}")
            return roboclaw0
        elif connected1:
            self.get_logger().info("Connected to /dev/serial1.")
            self.get_logger().info(f"version: {roboclaw1.ReadVersion(address)}")
            self.get_logger().info(f"encoders: {roboclaw1.ReadEncM1(address)}")
            return roboclaw1
        else:
            self.get_logger().warn("Could not open comport /dev/serial0 or /dev/serial1, make sure it has the correct permissions and is available")
            return None

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

        if self.color == 'red':
            self.get_logger().info(f'Spinning {self.number} times to the right')
            right = True
        elif self.color == 'blue':
            self.get_logger().info(f'Spinning {self.number} times to the left')
            right = False

        address1 = 128
        address2 = 129
        address3 = 130
        
        rc1 = self.test_connection(address1)
        rc2 = self.test_connection(address2)
        rc3 = self.test_connection(address3)

        ## Set accel
        drive_accel = 26213 # rul slow accel

        ## Set speed
        drive_vel = 14652 // 2

        motor_index = [15, 14, 13, 12]
        target_angle = [35, 125, 70, 70]

        kit = ServoKit(channels=16)
        sleep(0.1)
        for index, angle in zip(motor_index, target_angle):
            kit.servo[index].actuation_range = 300
            kit.servo[index].set_pulse_width_range(500, 2500)
            kit.servo[index].angle = angle
            self.get_logger().info(f'Servo motor at channel {index} was set to {angle}')
        
        if right:
            rc1.DutyAccelM1(address1, drive_accel, drive_vel)
            rc1.DutyAccelM2(address1, drive_accel, drive_vel)
            rc2.DutyAccelM1(address2, drive_accel, -drive_vel * -1)
            rc2.DutyAccelM2(address2, drive_accel, drive_vel)
            rc3.DutyAccelM1(address3, drive_accel, -drive_vel * -1)
            rc3.DutyAccelM2(address3, drive_accel, drive_vel * -1)
        else:
            rc1.DutyAccelM1(address1, drive_accel, drive_vel * -1)
            rc1.DutyAccelM2(address1, drive_accel, drive_vel * -1)
            rc2.DutyAccelM1(address2, drive_accel, -drive_vel)
            rc2.DutyAccelM2(address2, drive_accel, drive_vel * -1)
            rc3.DutyAccelM1(address3, drive_accel, -drive_vel)
            rc3.DutyAccelM2(address3, drive_accel, drive_vel)

        # number laps
        for i in range(self.number):
            self.get_logger().info(f'lap {i+1}')
            sleep(self.lap_time)

        # stop
        rc1.ForwardM1(address1, 0)
        rc1.ForwardM2(address1, 0)
        rc2.ForwardM1(address2, 0)
        rc2.ForwardM2(address2, 0)
        rc3.ForwardM1(address3, 0)
        rc3.ForwardM2(address3, 0)
        sleep(5)


def main():
    rclpy.init()
    perception_spin = PerceptionSpin()
    rclpy.spin(perception_spin)

    perception_spin.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
