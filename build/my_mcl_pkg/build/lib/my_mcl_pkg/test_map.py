#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy
import numpy as np

class MapLoaderNode(Node):
    def __init__(self):
        super().__init__('map_loader_node')
        
        # Subscribe to the /map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,       
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  
                history=HistoryPolicy.KEEP_LAST,                 
                depth=1                                          
            )
        )

        self.get_logger().info('Waiting for map data from /map topic...')

    def map_callback(self, msg):
        # Initialize the occupancy grid map
        ogm = msg
        try:
            # Convert map data to numpy array
            grid_map = np.array(ogm.data, dtype='int8').reshape((ogm.info.height, ogm.info.width))
            grid_bin = (grid_map == 0).astype('uint8')  # Cell is True iff probability of being occupied is zero
            self.get_logger().info(f"Grid bin: {grid_bin} m/cell")
            # Log map information
            self.get_logger().info(f"Received map: {ogm.info.width}x{ogm.info.height} cells")
            self.get_logger().info(f"Map resolution: {ogm.info.resolution} m/cell")
            self.get_logger().info(f"Map origin: ({ogm.info.origin.position.x}, {ogm.info.origin.position.y})")
            # Print entire grid map data (may be large)
            self.get_logger().info(f"ogm.info.origin pos x:{ogm.info.origin.position.x}")
            self.get_logger().info(f"ogm.info.origin posyx:{ogm.info.origin.position.y}")
          
            xmin = ogm.info.origin.position.x
            ymin = ogm.info.origin.position.y
            xmax = xmin + ogm.info.width * ogm.info.resolution
            ymax = ymin + ogm.info.height * ogm.info.resolution

            # Print boundaries
            self.get_logger().info(f"Map boundaries:")
            self.get_logger().info(f"xmin: {xmin}, xmax: {xmax}")
            self.get_logger().info(f"ymin: {ymin}, ymax: {ymax}")


            # for row in grid_map:
            #     self.get_logger().info(f"{row.tolist()}")

        except Exception as e:
            self.get_logger().error(f"Error processing map: {e}")
            self.get_logger().error(f"Error processing map: {e}")
            self.get_logger().error(f"Error processing map: {e}")
            self.get_logger().error(f"Error processing map: {e}")
            self.get_logger().error(f"Error processing map: {e}")
            self.get_logger().error(f"Error processing map: {e}")
            self.get_logger().error(f"Error processing map: {e}")
            self.get_logger().error(f"Error processing map: {e}")
            self.get_logger().error(f"Error processing map: {e}")
            self.get_logger().error(f"Error processing map: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapLoaderNode()
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
