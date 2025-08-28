#!/usr/bin/python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from simple_pid import PID
from adafruit_servokit import ServoKit
from time import sleep


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

        # Publisher
        # The Intuitive mode means that sending a positive angular velocity will always make the corner wheels turn regardless of the linear velocity.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_intuitive', 1)
        self.get_logger().info('LidarDetector publisher is UP')

        # PIDs
        self.pid_linear = PID(-0.25, -0.0, -0.2, setpoint=0.3)
        self.pid_linear.sample_time = 0.01
        self.pid_angular = PID(10, 0.5, 0.005, setpoint=0.0)
        self.pid_angular.sample_time = 0.01

        # servos
        self.motor_index = [15, 14, 13, 12]
        self.straight_angle = [90, 90, 95, 35]
        self.right_angle = [110, 60, 70, 60]
        self.left_angle = [60, 110, 115, 10]

        # set straight
        self.kit = ServoKit(channels=16)
        sleep(0.1)
        for index, angle in zip(self.motor_index, self.straight_angle):
            self.kit.servo[index].actuation_range = 300
            self.kit.servo[index].set_pulse_width_range(500, 2500)
            self.kit.servo[index].angle = angle
        
    def lidar_callback(self, msg):
        # get distances
        ranges = len(msg.ranges)

        left = np.array([msg.ranges[i] for i in range(ranges//4-5, ranges//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        left = left.mean()
        mid_left = np.array([msg.ranges[i] for i in range(ranges*3//8-5, ranges*3//8+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        mid_left = mid_left.mean()
        mid = np.array([msg.ranges[i] for i in range(ranges//2-5, ranges//2+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        mid = mid.mean()
        mid_right = np.array([msg.ranges[i] for i in range(ranges*5//8-5, ranges*5//8+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        mid_right = mid_right.mean()
        right = np.array([msg.ranges[i] for i in range(ranges*3//4-5, ranges*3//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        right = right.mean()

        # wall too far
        if np.isnan(left):
            left = 10
        if np.isnan(right):
            right = 10
        if np.isnan(mid_left):
            mid_left = 10
        if np.isnan(mid_right):
            mid_right = 10

        turn_left = left + mid_right
        turn_right = mid_left + right

        self.get_logger().info(f'left {turn_left.round(2)}, mid {mid.round(2)}, right {turn_right.round(2)}')

        # turn servos
        if turn_left > 3.5 and mid < 1.5 and self.kit.servo[0].angle != self.left_angle[0]:
            for index, angle in zip(self.motor_index, self.left_angle):
                self.kit.servo[index].angle = angle
            
        elif turn_right > 3.5 and mid < 1.5 and self.kit.servo[0].angle != self.right_angle[0]:
            for index, angle in zip(self.motor_index, self.right_angle):
                self.kit.servo[index].angle = angle

        elif self.kit.servo[0].angle != self.straight_angle[0]:
            for index, angle in zip(self.motor_index, self.straight_angle):
                self.kit.servo[index].angle = angle

        # publish pid outputs
        pid_output_1 = self.pid_linear(mid)
        pid_output_2 = self.pid_angular(left-right)
        
        twist_msg = Twist()
        twist_msg.linear.x = pid_output_1
        if pid_output_1 < 0.2:
            # rotate in place
            twist_msg.angular.y = pid_output_2
        else:
            twist_msg.angular.z = pid_output_2
        # self.get_logger().info(f'linear {pid_output_1}, angular {pid_output_2}')
        self.publisher_.publish(twist_msg)


def main():
    rclpy.init()
    lidar_detector = LidarDetector()
    rclpy.spin(lidar_detector)

    lidar_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
