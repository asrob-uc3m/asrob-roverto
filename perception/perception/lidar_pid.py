#!/usr/bin/python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from simple_pid import PID


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
        # TODO: ajustar valores
        self.pid_linear = PID(-0.25, -0.0, -0.2, setpoint=0.3)
        self.pid_linear.sample_time = 0.01
        self.pid_angular = PID(-0.43, -0.0, -0.458, setpoint=0.0)
        self.pid_angular.sample_time = 0.01
        
    def lidar_callback(self, msg):
        # get distances
        ranges = len(msg.ranges)

        left = np.array([msg.ranges[i] for i in range(ranges//4-5, ranges//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        mid = np.array([msg.ranges[i] for i in range(ranges//2-5, ranges//2+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        right = np.array([msg.ranges[i] for i in range(ranges*3//4-5, ranges*3//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])

        # publish pid outputs
        pid_output_1 = self.pid_linear(mid)
        pid_output_2 = self.pid_angular(left-right)
        
        twist_msg = Twist()
        twist_msg.linear.x = pid_output_1
        twist_msg.angular.z = pid_output_2
        self.publisher_.publish(twist_msg)


def main():
    rclpy.init()
    lidar_detector = LidarDetector()
    rclpy.spin(lidar_detector)

    lidar_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
