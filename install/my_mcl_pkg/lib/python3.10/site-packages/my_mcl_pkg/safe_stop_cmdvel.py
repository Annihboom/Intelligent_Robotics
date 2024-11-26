#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SafetyDetectorNode(Node):
    def __init__(self):
        super().__init__('safety_detector_node')
        self.declare_parameter('height', 1.0)
        self.declare_parameter('width', 2.0)

        # Retrieve parameters
        self.height = self.get_parameter('height').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().double_value

        # Calculate line equation variables
        self.x1 = self.width / 2
        self.y2 = self.height
        self.m = self.y2 / self.x1  # Slope

        self.drive_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.safety_check, 10)

    def move(self, command):
        data = Twist()
        if not command:
            data.linear.x = 0.2  # Move robot at constant 0.2 m/s speed
        else:
            data.linear.x = 0.0  # Stop robot
            self.get_logger().info('OBSTACLE AHEAD, STOP!!!')

        # Publish data on cmd_vel topic
        self.drive_pub.publish(data)

    def safety_check(self, msg):
        # Check for obstacles
        is_obstacle = False
        len_scan = len(msg.ranges)
        half_len = len_scan // 2

        # Calculate laser scan sensor sampling resolution
        vertical_resolution = self.height / half_len

        for i in range(half_len):
            x = (self.y2 - (vertical_resolution * (half_len - 1 - i))) / self.m

            if msg.ranges[half_len - 1] < self.height:
                is_obstacle = True
                self.get_logger().info('OBSTACLE ON FRONT SIDE')
                break

            if msg.ranges[half_len - 1 - i] < x:
                is_obstacle = True
                self.get_logger().info('OBSTACLE ON RIGHT SIDE')
                break

            if msg.ranges[half_len + i] < x:
                is_obstacle = True
                self.get_logger().info('OBSTACLE ON LEFT SIDE')
                break

        self.move(is_obstacle)

def main(args=None):
    rclpy.init(args=args)
    safety_detector_node = SafetyDetectorNode()
    rclpy.spin(safety_detector_node)
    safety_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
