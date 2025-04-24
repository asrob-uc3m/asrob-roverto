#!/usr/bin/python3

import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from custom_msg.msg import Aruco
from rclpy.qos import qos_profile_sensor_data
from simple_pid import PID


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

        # distances
        self.left = 0
        self.right = 0
        self.mid = 0

        # pids
        self.pid_linear = PID(-0.25, -0.0, -0.2, setpoint=0.0)
        self.pid_linear.sample_time = 0.01
        self.pid_angular = PID(10, 0.5, 0.005, setpoint=0.0)
        self.pid_angular.sample_time = 0.01

        best_path = self.get_best_path()
        self.waypoint_navigation(best_path)

        
    def get_best_path(self):
        waypoints = self.waypoint_positions
        loc = (0, 0)
        best_path = []

        while len(waypoints) > 0:
            distances = [(wp[0], wp[1], math.dist(loc, wp)) for wp in waypoints]
            next_goal = min(distances, key = lambda d: d[2])[:-1]
            best_path.append(next_goal)
            waypoints.remove(next_goal)
            loc = next_goal

        return best_path

    def lidar_callback(self, msg):
        # get distances
        ranges = len(msg.ranges)

        left = np.array([msg.ranges[i] for i in range(ranges//4-5, ranges//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.left = left.mean()
        mid_left = np.array([msg.ranges[i] for i in range(ranges*3//8-5, ranges*3//8+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.mid_left = mid_left.mean()
        mid = np.array([msg.ranges[i] for i in range(ranges//2-5, ranges//2+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.mid = mid.mean()
        mid_right = np.array([msg.ranges[i] for i in range(ranges*5//8-5, ranges*5//8+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.mid_right = mid_right.mean()
        right = np.array([msg.ranges[i] for i in range(ranges*3//4-5, ranges*3//4+5) if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min])
        self.right = right.mean()

    def aruco_callback(self, msg):
        ids = msg.id
        angs = msg.angle
        dists = msg.distance

        # compute location based on arucos on sight, distances and angles
        loc_x = np.array()
        loc_y = np.array()
        for i in range(len(ids)):
            loc_x.append(self.aruco_positions[i][0] + dists[i] * math.cos(angs[i]))
            loc_y.append(self.aruco_positions[i][1] + dists[i] * math.sin(angs[i]))

        self.location = (loc_x.mean(), loc_y.mean())

    def waypoint_navigation(self, best_path):
        thres = 0.1

        for goal in best_path:
            dist_to_goal = 10

            while dist_to_goal > thres:
                dist_to_goal = math.dist(goal, self.location)
                ang_to_goal = math.acos((goal[0] - self.location[0]) / dist_to_goal)
                obstacle = self.mid < 0.3 or self.mid_right < 0.3 or self.mid_left < 0.3

                # considering location and goal, move
                if not obstacle:
                    pid_output_1 = self.pid_linear(dist_to_goal)
                    pid_output_2 = self.pid_angular(ang_to_goal)
                    
                    twist_msg = Twist()
                    twist_msg.linear.x = pid_output_1
                    if pid_output_1 < 0.2:
                        # rotate in place
                        twist_msg.angular.y = pid_output_2
                    else:
                        twist_msg.angular.z = pid_output_2
                    self.publisher_.publish(twist_msg)

                else:
                    # just obstacles forward for the moment
                    twist_msg = Twist()
                    twist_msg.linear.x = -0.1
                    if self.mid_right < 0.3:
                        twist_msg.angular.y = -1.75
                    elif self.mid_left < 0.3:
                        twist_msg.angular.y = 1.75
                    self.publisher_.publish(twist_msg)

            # stop
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.y = 0
            twist_msg.angular.z = 0
            self.publisher_.publish(twist_msg)
            time.sleep(5)


def main():
    rclpy.init()
    waypoint_navigation = WaypointNavigation()
    rclpy.spin(waypoint_navigation)

    waypoint_navigation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
