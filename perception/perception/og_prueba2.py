import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import simple_pid
import math


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


    def lidar_listener_callback_1(self, msg):
        self.ranges = [10.0 if r == 0 else r for r in msg.ranges]
        ranges = self.ranges
        num_ranges = len(ranges)  
        
        self.angle_increment = msg.angle_increment
        angle_increment = self.angle_increment
            
        angle_wings_rad = self.angle_wings
        
        # Calculate d_left
        left_start_angle = 2 * num_ranges // 9 - int(angle_wings_rad / angle_increment)
        left_end_angle = 2 * num_ranges // 9 + int(angle_wings_rad / angle_increment)
        left_ranges = ranges[left_start_angle:left_end_angle]

        d_left = min(sum(left_ranges) / len(left_ranges), 2)
        
        # Calculate d_right (average of the smallest 5 ranges within angle_wings of 270 degrees)
        right_start_angle = 7 * num_ranges // 9 - int(angle_wings_rad / angle_increment)
        right_end_angle = 7 * num_ranges // 9 + int(angle_wings_rad / angle_increment)       
        right_ranges = ranges[right_start_angle:right_end_angle]
        
        d_right = min(sum(right_ranges) / len(right_ranges), 2)

        # Calculate d_front
        d_front_0 = (ranges[1]+ranges[2]+ranges[-2] + ranges[-1]) / 4.0

        self.get_logger().info(f'left {d_left.round(2)}, mid {d_front_0.round(2)}, right {d_right.round(2)}')
        
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
