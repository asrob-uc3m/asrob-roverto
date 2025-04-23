#!/usr/bin/python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from custom_msg.msg import Aruco
from rclpy.qos import qos_profile_sensor_data
from simple_pid import PID
from filterpy.kalman import ExtendedKalmanFilter


class WaypointNavigation(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')

        # Subscriber
        self.subscription = self.create_subscription(LaserScan, 
                                                     '/scan',
                                                     self.lidar_callback,
                                                     qos_profile=qos_profile_sensor_data)
        self.subscription = self.create_subscription(Aruco,
                                                     '/aruco_topic',
                                                     self.aruco_callback)

        self.subscription
        self.get_logger().info('WaypointNavigation subscriber is UP')

        # Publisher
        # The Intuitive mode means that sending a positive angular velocity will always make the corner wheels turn regardless of the linear velocity.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_intuitive', 1)
        self.get_logger().info('WaypointNavigation publisher is UP')

        # ArUco and waypoints
        self.aruco_positions = {}
        self.waypoint_positions = []
        self.location = (0, 0)
        self.velocity = 0
        self.filter = ExtendedKalmanFilter(dim_x=3, dim_z=3)
        self.filter.x = self.location + self.velocity   # state estimate vector
        self.filter.P = np.eye(3) * 0.1                 # covariance matrix
        self.filter.R = np.eye(3) * 0.1                 # measurement noise matrix
        self.filter.Q = np.eye(3) * 0.3                 # process noise matrix

        # distances
        self.left = 0
        self.right = 0
        self.mid = 0

        
    def lidar_callback(self, msg):
        # get distances
        ranges = len(msg.ranges)

        left = np.array([msg.ranges[i] for i in range(ranges//4-5, ranges//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.left = left.mean()
        mid = np.array([msg.ranges[i] for i in range(ranges//2-5, ranges//2+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.mid = mid.mean()
        right = np.array([msg.ranges[i] for i in range(ranges*3//4-5, ranges*3//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.right = right.mean()

    def aruco_callback(self, msg):
        id = msg.id
        dist = msg.dist

        # compute location based on arucos on sight and distances
        z = [dist[0], dist[1], ]


    def waypoint_navigation(self):
        distances = [(wp[0], wp[1], math.dist(self.location, wp)) for wp in self.waypoint_positions]
        next_goal = min(distances, key = lambda d: d[2])[:-1]

        # considering location and goal, move. If obstacle, go around and continue

def main():
    rclpy.init()
    waypoint_navigation = WaypointNavigation()
    rclpy.spin(waypoint_navigation)

    waypoint_navigation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
