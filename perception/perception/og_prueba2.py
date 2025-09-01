import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import simple_pid
import math
import numpy as np


class pid_controller_node(Node):   ##Node for Laser Scan subscription
    def __init__(self):
        super().__init__('pid_controller_node')
        
        # Subscriber
        self.subscription = self.create_subscription(LaserScan, 
                                                     '/scan',
                                                     self.lidar_callback,
                                                     qos_profile=qos_profile_sensor_data)
        self.subscription
        self.get_logger().info('LidarDetector subscriber is UP')

        # Twist publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_intuitive', 10)
        self.get_logger().info('LidarDetector publisher is UP')
        
        # Input variables for the PID controllers.        
        self.d_front = None
        self.d_sides = None
        
        self.angle_wings = math.radians(10) 
        self.d_safety=0.3
        
        # PID Controllers        
        self.pid_linear = simple_pid.PID(-0.25, -0.0, -0.2, setpoint=self.d_safety) 
        self.pid_angular = simple_pid.PID(-0.43, -0.0, -0.458, setpoint=0.0)
        
        # Timer to update the twist msgs.
        self.timer = self.create_timer(0.025, self.update_and_publish)


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
        d_left = min(turn_left, 3)
        d_right = min(turn_right, 3)
        d_front_0 = mid

        self.get_logger().info(f'left {d_left}, mid {d_front_0}, right {d_right}')
        
        # Update class variables
        self.d_front = min(d_front_0, 2.5)
        self.d_sides = d_left - d_right


    def update_and_publish(self):
        if self.d_front is not None and self.d_sides is not None:
            # Compute the PID outputs.
            pid_output_1 = self.pid_linear(self.d_front)
            pid_output_2 = self.pid_angular(self.d_sides)
            
            # Generate and publish new twist command.
            twist_msg = Twist()
            twist_msg.linear.x = pid_output_1
            twist_msg.angular.z = pid_output_2
            self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = pid_controller_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
