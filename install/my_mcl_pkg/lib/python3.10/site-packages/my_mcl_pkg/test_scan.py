#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserScanTester(Node):
    def __init__(self):
        super().__init__('laser_scan_tester')
        
        # Subscribe to the /scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/bcr_bot/scan',  
            self.scan_callback,
            10  # QoS profile
        )
        self.get_logger().info('Subscribed to /scan topic, waiting for laser scan data...')

    def scan_callback(self, msg):
        try:
            # Extract and log basic LaserScan information
            self.get_logger().info(f"Laser scan received:")
            self.get_logger().info(f"Angle range: [{msg.angle_min:.2f}, {msg.angle_max:.2f}] radians")
            self.get_logger().info(f"Angle increment: {msg.angle_increment:.4f} radians")
            self.get_logger().info(f"Range min: {msg.range_min:.2f} m, Range max: {msg.range_max:.2f} m")
            self.get_logger().info(f"Number of beams: {len(msg.ranges)}")

            # Print a subset of ranges for debugging
            subset_ranges = msg.ranges[::len(msg.ranges)//10]  # Take 10 evenly spaced beams
            subset_ranges = [r if np.isfinite(r) else "Inf" for r in subset_ranges]
            self.get_logger().info(f"Sampled ranges: {subset_ranges}")

            # Debug: Check for invalid data
            if all(np.isinf(msg.ranges)):
                self.get_logger().warn("All ranges are Inf! Check the laser scanner or topic publisher.")

        except Exception as e:
            self.get_logger().error(f"Error processing laser scan: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanTester()
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
