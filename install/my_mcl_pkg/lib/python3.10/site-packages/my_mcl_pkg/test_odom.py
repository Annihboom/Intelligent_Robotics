#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomTester(Node):
    def __init__(self):
        super().__init__('odom_tester')

        # Subscribe to the /odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            'bcr_bot/odom',  
            self.odom_callback,
            10  # QoS profile
        )
        self.get_logger().info('Subscribed to /odom topic, waiting for odometry data...')

    def odom_callback(self, msg):
        try:
            # Extract and log key odometry information
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            linear_velocity = msg.twist.twist.linear
            angular_velocity = msg.twist.twist.angular

            # Convert orientation quaternion to Euler angles
            yaw = self.quaternion_to_yaw(orientation)

            # Log odometry data
            self.get_logger().info(f"Position: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}")
            self.get_logger().info(f"Orientation (Yaw): {math.degrees(yaw):.2f} degrees")
            self.get_logger().info(f"Linear Velocity: x={linear_velocity.x:.3f}, y={linear_velocity.y:.3f}, z={linear_velocity.z:.3f}")
            self.get_logger().info(f"Angular Velocity: x={angular_velocity.x:.3f}, y={angular_velocity.y:.3f}, z={angular_velocity.z:.3f}")

        except Exception as e:
            self.get_logger().error(f"Error processing odometry data: {e}")

    def quaternion_to_yaw(self, orientation):
        """
        Converts a quaternion to a yaw angle (in radians).
        """
        # Extract quaternion components
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w

        # Compute yaw from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
