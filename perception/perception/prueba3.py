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
from scipy.optimize import least_squares


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
                                                     self.aruco_callback,
                                                     qos_profile=qos_profile_sensor_data)

        self.subscription
        self.get_logger().info('WaypointNavigation subscriber is UP')

        # Publisher
        # The Intuitive mode means that sending a positive angular velocity will always make the corner wheels turn regardless of the linear velocity.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_intuitive', 1)
        self.get_logger().info('WaypointNavigation publisher is UP')

        # ArUco and waypoints
        self.aruco_positions = {20: (1.5, 7), 25: (3.5, 7), 28: (5.5, 7),
                                40: (0, 1.5), 44: (0, 3.5), 49: (0, 5.5),
                                60: (1.5, 0), 62: (3.5, 0), 67: (5.5, 0),
                                90: (7, 1.5), 93: (7, 3.5), 96: (7, 5.5)}
        self.waypoint_positions = [(1.5, 1.5), (5, 2.5), (3, 3), (2, 5.5), (5.5, 5.5)]
        self.location = (0, 0)

        # distances
        self.left = 0
        self.right = 0
        self.mid = 0
        self.mid_right = 0
        self.mid_left = 0

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
        
    def intersectionPoint(p1, p2, p3=None):
        # https://stackoverflow.com/questions/60555205/in-python-how-to-get-intersection-point-of-three-or-more-circles-with-or-without
        if p3 == None:
            p3 = p2

        x1, y1, dist_1 = (p1[0], p1[1], p1[2])
        x2, y2, dist_2 = (p2[0], p2[1], p2[2])
        x3, y3, dist_3 = (p3[0], p3[1], p3[2])

        def eq(g):
            x, y = g
            return (
                (x - x1)**2 + (y - y1)**2 - dist_1**2,
                (x - x2)**2 + (y - y2)**2 - dist_2**2,
                (x - x3)**2 + (y - y3)**2 - dist_3**2)

        guess = (x1, y1 + dist_1)
        ans = least_squares(eq, guess, ftol=None, xtol=None)

        return ans

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
        if len(ids) == 0:
            loc_x, loc_y = self.location
        elif len(ids) == 1:
            loc_x = self.aruco_positions[ids[0]][0] + dists[0] * math.cos(angs[0])
            loc_y = self.aruco_positions[ids[0]][1] + dists[0] * math.sin(angs[0])
        elif len(ids) == 2:
            ans = self.intersectionPoint((self.aruco_positions[ids[0]][0], self.aruco_positions[ids[0]][1], dists[0]), 
                                         (self.aruco_positions[ids[1]][0], self.aruco_positions[ids[1]][1], dists[1]))
            loc_x, loc_y = ans.x
        else:
            ans = self.intersectionPoint((self.aruco_positions[ids[0]][0], self.aruco_positions[ids[0]][1], dists[0]), 
                                         (self.aruco_positions[ids[1]][0], self.aruco_positions[ids[1]][1], dists[1]),
                                         (self.aruco_positions[ids[2]][0], self.aruco_positions[ids[2]][1], dists[2]))
            loc_x, loc_y = ans.x

        self.location = (loc_x, loc_y)

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
