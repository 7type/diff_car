#!/usr/bin/env python3
"""
简单演示导航节点
不用A*，直接写死一条绕障路径，适合快速验证底盘控制。
航点手动调过，保证能绕过(3, 0)的障碍物。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time


class DemoSimpleNavigator(Node):
    def __init__(self):
        super().__init__('demo_simple_navigator')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
        #/odom是Gazebo差速驱动插件发布的，/diff_cont/odom是ros2_control的DiffDriveController发布的
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_sub2 = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        
        #发布goal marker方便在rviz里看到当前目标点
        self.goal_pub = self.create_publisher(PoseStamped, '/current_goal', 10)
        
        self.current_pose = None
        self.current_yaw = 0.0
        
        #航点是手动调出来的，y方向先负后正，形成一个弧形绕过障碍物
        self.waypoints = [
            (0.5, 0.0),
            (1.0, 0.0),
            (1.5, -0.2),
            (2.0, -0.4),
            (2.5, -0.6),
            (3.0, -0.7),
            (3.5, -0.6),
            (4.0, -0.4),
            (4.5, -0.2),
            (5.0, 0.0),
            (5.5, 0.0),
            (6.0, 0.0),
        ]
        self.current_waypoint_idx = 0
        
        #速度跟A*模式保持一致
        self.linear_speed = 0.15
        self.angular_speed = 0.3
        self.position_tolerance = 0.15
        
        #PI参数跟A*模式用同一套，方便对比效果
        self.kp = 2.5
        self.ki = 0.3
        self.integral_max = 1.0
        self.integral_min = -1.0
        self.integral_separation_threshold = 0.2
        self.integral = 0.0
        
        self.get_logger().info('Simple navigator started')
        self.get_logger().info('Waiting for odometry...')
        
        while self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().info('Odometry received, starting navigation in 3 seconds')
        #多等一会让Gazebo完全稳定下来
        time.sleep(3.0)
        
        self.run_navigation()
        
    def odom_callback(self, msg):
        #只在第一次收到时打印，避免日志刷屏
        if self.current_pose is None:
            self.get_logger().info(f'First odometry received: frame_id={msg.header.frame_id}')
        self.current_pose = msg.pose.pose
        q = msg.pose.pose.orientation
        #差速底盘只需要yaw角来控制朝向
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
    def get_yaw_to_target(self, target_x, target_y):
        if self.current_pose is None:
            return 0.0
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        return math.atan2(dy, dx)
        
    def normalize_angle(self, angle):
        #角度限制在[-π, π]，不然跨越±π时误差会突变
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def pi_control(self, error, dt=0.05):
        """
        PI控制器，差速底盘转向够用了，不需要D项。
        积分分离防止误差大时积分饱和。
        """
        #误差大时不用积分，避免饱和
        if abs(error) > self.integral_separation_threshold:
            integral_term = 0.0
        else:
            self.integral += error * dt
            
            #限幅防止积分失控
            if self.integral > self.integral_max:
                self.integral = self.integral_max
            elif self.integral < self.integral_min:
                self.integral = self.integral_min
            
            integral_term = self.ki * self.integral
        
        output = self.kp * error + integral_term
        
        #输出不能超过底盘最大角速度
        if output > self.angular_speed:
            output = self.angular_speed
        elif output < -self.angular_speed:
            output = -self.angular_speed
        
        return output
        
    def distance_to_target(self, target_x, target_y):
        if self.current_pose is None:
            return float('inf')
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        return math.sqrt(dx * dx + dy * dy)
        
    def publish_goal_marker(self, x, y):
        #发个PoseStamped给rviz，这样能看到当前要去哪
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        
    def run_navigation(self):
        self.get_logger().info(f'Starting navigation with {len(self.waypoints)} waypoints')

        while self.current_waypoint_idx < len(self.waypoints) and rclpy.ok():
            target_x, target_y = self.waypoints[self.current_waypoint_idx]
            
            #每个新航点重置积分，避免累积误差影响下一个点
            self.integral = 0.0
            
            self.get_logger().info(
                f'Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
                f'({target_x:.2f}, {target_y:.2f})'
            )
            
            self.publish_goal_marker(target_x, target_y)
            
            while self.distance_to_target(target_x, target_y) > self.position_tolerance:
                if self.current_pose is None:
                    continue
                    
                target_yaw = self.get_yaw_to_target(target_x, target_y)
                yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
                
                cmd = Twist()
                
                #偏差大时先原地转，偏差小了再边前进边微调
                if abs(yaw_error) > 0.5:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.angular_speed if yaw_error > 0 else -self.angular_speed
                    self.integral = 0.0
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = self.pi_control(yaw_error)
                    
                self.cmd_vel_pub.publish(cmd)
                
                rclpy.spin_once(self, timeout_sec=0.05)
                
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx + 1}')
            self.current_waypoint_idx += 1
            
        self.get_logger().info('Navigation complete!')
        
        #走完了停住，不然会一直往前溜
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DemoSimpleNavigator()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
