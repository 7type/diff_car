#!/usr/bin/env python3
"""
A*寻路导航节点
地图是80x80的栅格，每个格子0.2m，平衡精度和性能。
障碍物放在(3, 0)能测试绕障能力。

"""
from __future__ import annotations
from typing import Dict, Tuple, List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time
import heapq


class AStarNavigator(Node):
    def __init__(self):
        super().__init__('astar_navigator')

        #发布到diff_cont的cmd_vel topic，这是差速控制器期望的输入
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        #订阅两个odom topic是为了兼容不同配置：
        #/odom是Gazebo差速驱动插件发布的，/diff_cont/odom是ros2_control的DiffDriveController发布的
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_sub2 = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)

        self.current_pose = None
        self.current_yaw = 0.0

        #起点和终点故意放在x轴上，中间放个障碍物，这样就能看出A*会不会绕路
        self.start = (0.0, 0.0)
        self.goal = (6.0, 0.0)

        #速度设得比较保守，仿真里跑太快容易overshoot，越过设定
        self.linear_speed = 0.15
        self.angular_speed = 0.3
        self.position_tolerance = 0.15

        #kp太大容易震荡，太小又转不到位；ki用来消除稳态误差
        self.kp = 2.5
        self.ki = 0.3
        self.integral_max = 1.0
        self.integral_min = -1.0
        self.integral_separation_threshold = 0.2
        self.integral = 0.0

        #地图参数：原点在(-4, -4)是为了让(0, 0)在地图中间偏左的位置
        #这样6m的终点也能落在地图范围内
        self.map_width = 80
        self.map_height = 80
        self.map_resolution = 0.2
        self.map_origin_x = -4.0
        self.map_origin_y = -4.0
        self.robot_radius = 0.3

        self.grid_map = [[0] * self.map_height for _ in range(self.map_width)]

        #初始化时就加上障碍物，实际应用中应该从地图topic动态获取
        self.add_obstacle(3.0, 0.0, 0.3)

        self.get_logger().info('A* Navigator started')
        self.get_logger().info('Waiting for odometry...')

        #阻塞等待第一帧里程计，没有里程计就没法导航
        while self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Odometry received, computing A* path...')

        path = self.compute_astar_path()

        if path is None:
            self.get_logger().error('No path found!')
            return

        self.get_logger().info(f'Path found with {len(path)} waypoints')

        #延迟2秒再出发，给rviz或其他可视化工具一点时间更新
        time.sleep(2.0)

        self.follow_path(path)

    def odom_callback(self, msg):
        #只在第一次收到里程计时打印日志，避免刷屏
        if self.current_pose is None:
            self.get_logger().info(f'First odometry received: frame_id={msg.header.frame_id}')
        self.current_pose = msg.pose.pose
        q = msg.pose.pose.orientation
        #从四元数转yaw角，差速底盘只需要这个角度来控制朝向
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def world_to_grid(self, wx, wy):
        #世界坐标转栅格索引，减去原点偏移再除以分辨率就行
        gx = int((wx - self.map_origin_x) / self.map_resolution)
        gy = int((wy - self.map_origin_y) / self.map_resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        #栅格转世界坐标时加上半个格子的偏移，让坐标落在格子中心
        wx = gx * self.map_resolution + self.map_origin_x + self.map_resolution / 2
        wy = gy * self.map_resolution + self.map_origin_y + self.map_resolution / 2
        return wx, wy

    def add_obstacle(self, ox, oy, radius):
        #把障碍物从世界坐标转成栅格坐标
        ox_g, oy_g = self.world_to_grid(ox, oy)
        r_g = int(radius / self.map_resolution) + 1
        #加上机器人半径的膨胀余量，不然规划出来的路径会贴着障碍物走，实际跑的时候容易撞上
        margin = int(self.robot_radius / self.map_resolution) + 1
        total_r = r_g + margin

        #用圆形膨胀而不是方形，这样障碍物边缘更平滑
        for dx in range(-total_r, total_r + 1):
            for dy in range(-total_r, total_r + 1):
                if dx * dx + dy * dy <= total_r * total_r:
                    gx, gy = ox_g + dx, oy_g + dy
                    if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                        self.grid_map[gx][gy] = 100

    def is_safe(self, gx, gy):
        #越界或者碰到障碍物格子都算不安全
        if gx < 0 or gx >= self.map_width or gy < 0 or gy >= self.map_height:
            return False
        return self.grid_map[gx][gy] == 0

    def heuristic(self, a, b):
        #用欧氏距离做启发函数，因为允许8方向移动
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def compute_astar_path(self):
        start_g = self.world_to_grid(self.start[0], self.start[1])
        goal_g = self.world_to_grid(self.goal[0], self.goal[1])

        if not self.is_safe(*start_g):
            self.get_logger().warn('Start position is not safe')
        if not self.is_safe(*goal_g):
            self.get_logger().warn('Goal position is not safe')

        #用优先队列存待扩展的节点，f值小的优先
        open_set = []
        heapq.heappush(open_set, (0, start_g))
        came_from = {}
        g_score: Dict[Tuple[int, int], float] = {start_g: 0.0}
        f_score: Dict[Tuple[int, int], float] = {start_g: self.heuristic(start_g, goal_g)}

        #8方向移动：上下左右+四个对角线，对角线代价设为1.414近似sqrt(2)
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, current = heapq.heappop(open_set)

            #到达目标，回溯路径
            if current == goal_g:
                path = []
                node = current
                while node in came_from:
                    wx, wy = self.grid_to_world(*node)
                    path.append((wx, wy))
                    node = came_from[node]
                wx, wy = self.grid_to_world(*start_g)
                path.append((wx, wy))
                path.reverse()
                return path

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.is_safe(*neighbor):
                    continue

                #对角线移动代价更高，这样算法会倾向于走直线而不是zigzag
                move_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                tentative_g = g_score[current] + move_cost

                #找到更短的路径就更新
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal_g)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))

        self.get_logger().error('A* search exhausted, no path found')
        return None

    def get_yaw_to_target(self, target_x, target_y):
        if self.current_pose is None:
            return 0.0
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        #atan2返回的是从当前点指向目标点的角度，用来计算机器人应该朝向哪
        return math.atan2(dy, dx)

    def normalize_angle(self, angle):
        #把角度限制在[-π, π]范围内，不然误差会越算越大
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def pi_control(self, error, dt=0.05):
        """
        PI控制器用来计算机器人的角速度

        为什么用PI而不是PID：差速底盘转向主要靠比例项，积分项只是用来消除稳态误差
        微分项在这里反而会引入噪声，所以干脆不用

        积分分离的设计思路：误差大的时候积分项只会帮倒忙，所以等误差小了才开始累积积分
        """
        #误差太大时先不用积分，避免积分饱和
        if abs(error) > self.integral_separation_threshold:
            integral_term = 0.0
        else:
            #误差小的时候才开始累积积分
            self.integral += error * dt
            
            #积分限幅是防止积分项无限增长，不然控制器会失控
            if self.integral > self.integral_max:
                self.integral = self.integral_max
            elif self.integral < self.integral_min:
                self.integral = self.integral_min
            
            integral_term = self.ki * self.integral
        
        #比例项负责快速响应，积分项负责消除稳态误差
        output = self.kp * error + integral_term
        
        #输出限幅：不能超过底盘能达到的最大角速度
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

    def follow_path(self, path):
        self.get_logger().info(f'Following path with {len(path)} waypoints')

        for i, (target_x, target_y) in enumerate(path):
            self.get_logger().info(
                f'Waypoint {i + 1}/{len(path)}: ({target_x:.2f}, {target_y:.2f})'
            )

            #每个新航点都重置积分，避免上一个航点的累积误差影响当前航点
            self.integral = 0.0

            #还没到目标点就继续循环
            while self.distance_to_target(target_x, target_y) > self.position_tolerance:
                if self.current_pose is None:
                    continue

                target_yaw = self.get_yaw_to_target(target_x, target_y)
                yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

                cmd = Twist()

                #朝向偏差大的时候先原地转过去，不然会走弧线
                if abs(yaw_error) > 0.5:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.angular_speed if yaw_error > 0 else -self.angular_speed
                    #原地转向时清零积分，因为这时候不需要积分项
                    self.integral = 0.0
                else:
                    #朝向差不多了就边前进边微调
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = self.pi_control(yaw_error)

                self.cmd_vel_pub.publish(cmd)

                #0.05s的循环周期，差不多20Hz，对这种低速导航够用了
                rclpy.spin_once(self, timeout_sec=0.05)

            self.get_logger().info(f'Reached waypoint {i + 1}')

        self.get_logger().info('Navigation complete!')

        #走完了发个零速度，不然机器人会一直往前溜
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigator()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
