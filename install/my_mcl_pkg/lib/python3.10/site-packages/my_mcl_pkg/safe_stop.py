import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading

"""
The safe stop for teleop receives bcr_bot/teleop_cmd_vel topic and take control of it 
It will stop automatically 2 seconds when it detect obstacles in dangerous distance and 
after that give temporary control back to teleop for 2 seconds,
after 2 secs if it still detect obstacle, 
it will  stop again for 2 secs and give control for teleop for 2secs...


If it does not detect any obstacles in dangerous range, nothing will happen
So basically it does not affect the process of safe movement of robot,
it just protect robot from dangerous scneario 


This may works  at the mapping stage when we don't know obstacles around 
"""

class SafetyDetectorNode(Node):
    def __init__(self):
        super().__init__('safety_detector_node')
        self.declare_parameter('height', 1.5)
        self.declare_parameter('width', 1.7)
        self.declare_parameter('stop_time', 2.0)  # 停止时间，单位：秒

        #get params
        self.height = self.get_parameter('height').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().double_value
        self.stop_time = self.get_parameter('stop_time').get_parameter_value().double_value

        #get dangerous range
        self.x1 = self.width / 2
        self.y2 = self.height
        self.m = self.y2 / self.x1  

        # use to publish velocity
        self.drive_pub = self.create_publisher(Twist, 'bcr_bot/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, 'bcr_bot/scan', self.safety_check, 10)
        self.teleop_sub = self.create_subscription(Twist, 'bcr_bot/teleop_cmd_vel', self.teleop_callback, 10)

        # 初始化变量
        self.teleop_cmd = Twist()
        self.obstacle_detected = False  # for obstacle detect

    def teleop_callback(self, msg):
        """deal with teleop velocity control"""
        self.teleop_cmd = msg

    def move(self):
        """publish velocity control"""
        if self.obstacle_detected:
            # detect obstacle stop the robot
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.drive_pub.publish(stop_cmd)
        else:
            # does not detect obstacle, teleop keep controling
            self.drive_pub.publish(self.teleop_cmd)

    def stop(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.drive_pub.publish(stop_cmd)

    def safety_check(self, msg):
        """scan callback to check if it is safe"""
      
        if self.obstacle_detected:
            self.start_stop_timer()
            # self.move()
            # self.create_timer(self.stop_time, self.passs)
            return  # skip if already detected obstacle 

        len_scan = len(msg.ranges)
        half_len = len_scan // 2
        vertical_resolution = self.height / half_len

        for i in range(half_len):
            x = (self.y2 - (vertical_resolution * (half_len - 1 - i))) / self.m

            # detect obstacle in dangerous distance
            if msg.ranges[half_len - 1] < self.height or \
                msg.ranges[half_len - 1 - i] < x or \
                msg.ranges[half_len + i] < x:
                self.obstacle_detected = True
                self.stop()
                self.get_logger().info('OBSTACLE DETECTED, STOPPING FOR 2 SECONDS')
                break

        self.move()
    
    def passs(self):
        pass

    def start_stop_timer(self):
        """start stop timer"""
        self.create_timer(self.stop_time, self.passs)
        self.restore_control()

    def start_move_timer(self):
        """start move timer"""
        self.get_logger().info("start move timer")
        self.create_timer(self.stop_time, self.move)

    def restore_control(self):
        """restore  teleop control after stop"""
        self.obstacle_detected = False  # temporarily clear state for moving bot from danger distance
        self.get_logger().info('Teleop control restored after STOP')
        self.start_move_timer()

def main(args=None):
    rclpy.init(args=args)
    safety_detector_node = SafetyDetectorNode()
    rclpy.spin(safety_detector_node)
    safety_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

