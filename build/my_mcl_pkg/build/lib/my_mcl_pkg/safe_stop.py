import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading

class SafetyDetectorNode(Node):
    def __init__(self):
        super().__init__('safety_detector_node')
        self.declare_parameter('height', 1.5)
        self.declare_parameter('width', 1.7)
        self.declare_parameter('stop_time', 2.0)  # 停止时间，单位：秒

        # 获取参数值
        self.height = self.get_parameter('height').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().double_value
        self.stop_time = self.get_parameter('stop_time').get_parameter_value().double_value

        # 计算线性方程的变量
        self.x1 = self.width / 2
        self.y2 = self.height
        self.m = self.y2 / self.x1  # 斜率

        # 用于发布速度命令
        self.drive_pub = self.create_publisher(Twist, 'bcr_bot/cmd_vel', 10)
        # 订阅激光扫描数据
        self.laser_sub = self.create_subscription(LaserScan, 'bcr_bot/scan', self.safety_check, 10)
        # 订阅 teleop 控制速度命令
        self.teleop_sub = self.create_subscription(Twist, 'bcr_bot/teleop_cmd_vel', self.teleop_callback, 10)

        # 初始化变量
        self.teleop_cmd = Twist()
        self.obstacle_detected = False  # 障碍物检测状态
        self.mutex = threading.Lock()  # 用于状态控制的锁

    def teleop_callback(self, msg):
        """处理 teleop 的速度控制命令"""
        self.teleop_cmd = msg

    def move(self):
        """发布运动控制指令"""
        if self.obstacle_detected:
            # 如果检测到障碍物，发送停止命令
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.drive_pub.publish(stop_cmd)
        else:
            # 如果未检测到障碍物，执行 teleop 控制
            self.drive_pub.publish(self.teleop_cmd)

    def stop(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.drive_pub.publish(stop_cmd)

    def safety_check(self, msg):
        """激光扫描数据回调，检测障碍物"""
      
        if self.obstacle_detected:
            self.start_stop_timer()
            # self.move()
            # self.create_timer(self.stop_time, self.passs)
            return  # 如果已经在障碍物状态，跳过处理

        len_scan = len(msg.ranges)
        half_len = len_scan // 2
        vertical_resolution = self.height / half_len

        for i in range(half_len):
            x = (self.y2 - (vertical_resolution * (half_len - 1 - i))) / self.m

            # 如果在指定区域检测到障碍物
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
        """启动停止计时器"""
        self.create_timer(self.stop_time, self.passs)
        self.restore_control()

    def start_move_timer(self):
        """启动停止计时器"""
        self.get_logger().info("start move timer")
        self.create_timer(self.stop_time, self.move)

    def restore_control(self):
        """计时器到期后恢复 teleop 控制"""
        self.obstacle_detected = False  # 清除障碍物状态
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

