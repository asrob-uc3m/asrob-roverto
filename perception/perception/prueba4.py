#!/usr/bin/python3

import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from custom_msg.msg import Aruco
from rclpy.qos import qos_profile_sensor_data
from simple_pid import PID


class SpaceMission(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')

        # Subscriber
        self.subscription1 = self.create_subscription(LaserScan, 
                                                     '/scan',
                                                     self.lidar_callback,
                                                     qos_profile=qos_profile_sensor_data)
        self.subscription2 = self.create_subscription(Aruco,
                                                     '/aruco_topic',
                                                     self.aruco_callback)
        self.subscription3 = self.create_subscription(String,
                                                     '/perception/number_topic',
                                                     self.number_callback, 
                                                     1)
        self.subscription4 = self.create_subscription(String,
                                                     '/perception/box_color_topic',
                                                     self.color_callback, 
                                                     1)

        self.subscription1
        self.subscription2
        self.subscription3
        self.subscription4
        self.get_logger().info('WaypointNavigation subscriber is UP')

        # Publisher
        # The Intuitive mode means that sending a positive angular velocity will always make the corner wheels turn regardless of the linear velocity.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_intuitive', 1)
        self.get_logger().info('WaypointNavigation publisher is UP')

        # Detection
        self.class_labels = {'uno': 1, 'dos': 2, 'tres': 3, 'cuatro': 4, 'cinco': 5, 'seis': 6, 'siete': 7, 'ocho': 8, 'nueve': 9}
        self.color, self.number = None, None
        self.aruco_count = 0

        # Spin
        self.pid_linear = PID(-0.25, -0.0, -0.2, setpoint=0.0)
        self.pid_linear.sample_time = 0.01
        self.pid_angular = PID(10, 0.5, 0.005, setpoint=0.0)
        self.pid_angular.sample_time = 0.01
        self.spin_rate = 2.0
        self.lap_time = 10
        self.turn = 0

        # ArUco and waypoints
        self.aruco_positions = {}
        self.waypoint_positions = []
        self.location = (0, 0)

        # distances
        self.left = 0
        self.right = 0
        self.mid = 0

        self.launch_space_mission()

        
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
        ids = msg.id
        angs = msg.angle
        dists = msg.distance

        # compute location based on arucos on sight and distances
        loc_x = np.array()
        loc_y = np.array()
        for i in range(len(ids)):
            if ids[i] in self.aruco_positions.keys():
                loc_x.append(self.aruco_positions[ids[i]][0] + dists[i] * math.cos(angs[i]))
                loc_y.append(self.aruco_positions[ids[i]][1] + dists[i] * math.sin(angs[i]))

        if len(loc_x) > 0:
            self.location = (loc_x.mean(), loc_y.mean())
        else:
            self.location = (0, 0)

        # count arucos
        # Only count if at a determined distance (otherwise can count multiple times)
        if self.color == None and 73 in ids and dists[ids.index(73)] == 1.0:
            self.aruco_count += 1
        elif (self.color == 'red' or self.color == 'blue') and self.number in ids and dists[ids.index(self.number)] == 1.0:
            self.aruco_count += 1

        # signal turns
        turn_possible = self.right > 1 and self.left > 1
        if 10 in ids and turn_possible:
            self.turn = 1   # turn left
        elif 11 in ids and turn_possible:
            self.turn = 2   # turn right
        elif 15 in ids and turn_possible:
            self.turn = 3   # stop
        else:
            self.turn = 0   # no turn
        

    def number_callback(self, msg):
        number_name = msg.data
        
        if number_name in self.class_labels.keys():
            self.number = self.class_labels[number_name]
        else:
            self.number = None

    def color_callback(self, msg):
        color_name = msg.data

        if color_name != 'none':
            self.color = color_name
        else:
            self.color = None

    def stop_rover(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher_.publish(twist)

    def launch_space_mission(self):
        # wait some while getting color and number info
        time.sleep(10)

        # close color and number subscriptions
        self.destroy_subscription(self.subscription3)
        self.destroy_subscription(self.subscription4)

        # spin once
        twist = Twist()
        twist.angular.y = self.spin_rate
        spinning_time = self.lap_time
        start_time = time.time()
        current_time = time.time()
        while current_time < start_time + spinning_time:
            self.publisher_.publish(twist)
            current_time = time.time()

        self.stop_rover()

        # localize and go to start
        goal = (0, 0)
        dist_to_goal = math.dist(goal, self.location)

        while dist_to_goal > 0.1:
            dist_to_goal = math.dist(goal, self.location)
            ang_to_goal = math.acos((goal[0] - self.location[0]) / dist_to_goal)

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

        self.stop_rover()

        # go forward unless turn aruco detected
        while self.turn != 3:
            if self.turn == 0:
                pid_output_1 = self.pid_linear(self.mid)
                twist.linear.x = pid_output_1
            if self.turn == 1:
                twist.angular.y = - self.spin_rate
                twist.linear.x = 0.1
            if self.turn == 2:
                twist.angular.y = self.spin_rate
                twist.linear.x = 0.1
            
            self.publisher_.publish(twist)

        self.stop_rover()

        # stop at 1.5m from wall
        while self.mid > 1.5:
            pid_output_1 = self.pid_linear(self.mid) / 2
            twist.linear.x = pid_output_1
            self.publisher_.publish(twist)

        self.stop_rover()

        # spin number of times
        if self.color == 'red' or self.color == None:
            twist.angular.y = self.spin_rate
        elif self.color == 'blue':
            twist.angular.y = - self.spin_rate
        
        spinning_time = self.lap_time * self.aruco_count
        start_time = time.time()
        current_time = time.time()
        while current_time < start_time + spinning_time:
            self.publisher_.publish(twist)
            current_time = time.time()

        self.stop_rover()


def main():
    rclpy.init()
    space_mission = SpaceMission()
    rclpy.spin(space_mission)

    space_mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()