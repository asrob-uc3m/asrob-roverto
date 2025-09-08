#!/usr/bin/python3

import rclpy
import numpy as np
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
        # ranges between [0.1, 12]
        ranges = len(msg.ranges)

        # values at 45 degree
        right = np.array([msg.ranges[i] for i in range(ranges//4-5, ranges//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.get_logger().info(f'Distance right: {right.mean()}')

        # values at 90 degree
        mid = np.array([msg.ranges[i] for i in range(ranges//2-5, ranges//2+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.get_logger().info(f'Distance front: {mid.mean()}')

        # values at 135 degree
        left = np.array([msg.ranges[i] for i in range(ranges*3//4-5, ranges*3//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.get_logger().info(f'Distance left: {left.mean()}')

        # values at 0-180 degree
        back = [msg.ranges[i] for i in range(ranges-5, ranges) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min]
        back += [msg.ranges[i] for i in range(0, 5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min]
        back = np.array(back)
        self.get_logger().info(f'Distance back: {back.mean()}')


def main():
    rclpy.init()
    lidar_detector = LidarDetector()
    rclpy.spin(lidar_detector)

    lidar_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
