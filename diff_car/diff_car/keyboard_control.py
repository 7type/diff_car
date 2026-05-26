#!/usr/bin/env python3
"""
键盘控制节点
用S曲线插值让速度变化平滑，不然按下去就是满速，仿真里容易失控。
五次多项式保证加速度连续，比简单的线性ramp舒服很多。

启动: ros2 run diff_car keyboard_control
"""
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
        #速度跟导航模式保持一致，手感统一
        self.max_linear_speed = 0.15
        self.max_angular_speed = 0.3
        
        #当前速度
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        #目标速度
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        #0.3s的加速时间，太短了会突兀，太长了又觉得肉
        self.accel_time = 0.3
        self.dt = 0.01
        
        #线速度插值状态
        self.start_linear = 0.0
        self.t_linear = 0.0
        self.is_accelerating_linear = False
        
        #角速度插值状态
        self.start_angular = 0.0
        self.t_angular = 0.0
        self.is_accelerating_angular = False
        
        self.get_logger().info('Keyboard control node started (S-curve interpolation)')
        self.get_logger().info('Use i/w: forward, ,/s: backward, j/a: left, l/d: right, k: stop, q: quit')
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            self.run_keyboard_control()
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            #退出时恢复终端设置并停车，不然终端会坏掉
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def s_curve_interpolation(self, start_value, target_value, t):
        """
        五次多项式S曲线，首尾的速度和加速度都是0，过渡特别平滑。
        用这个而不是线性插值是因为机器人惯性大，突变速度会打滑。
        """
        t = max(0.0, min(1.0, t))
        ratio = 10 * t**3 - 15 * t**4 + 6 * t**5
        return start_value + (target_value - start_value) * ratio
    
    def update_speed(self):
        #线速度更新
        if self.is_accelerating_linear:
            self.t_linear += self.dt / self.accel_time
            
            if self.t_linear >= 1.0:
                self.current_linear = self.target_linear
                self.is_accelerating_linear = False
            else:
                self.current_linear = self.s_curve_interpolation(
                    self.start_linear, self.target_linear, self.t_linear
                )
        else:
            #目标变了就开始新一轮插值
            if abs(self.target_linear - self.current_linear) > 0.001:
                self.start_linear = self.current_linear
                self.t_linear = 0.0
                self.is_accelerating_linear = True
        
        #角速度更新，逻辑跟线速度一样
        if self.is_accelerating_angular:
            self.t_angular += self.dt / self.accel_time
            
            if self.t_angular >= 1.0:
                self.current_angular = self.target_angular
                self.is_accelerating_angular = False
            else:
                self.current_angular = self.s_curve_interpolation(
                    self.start_angular, self.target_angular, self.t_angular
                )
        else:
            if abs(self.target_angular - self.current_angular) > 0.001:
                self.start_angular = self.current_angular
                self.t_angular = 0.0
                self.is_accelerating_angular = True
    
    def run_keyboard_control(self):
        while rclpy.ok():
            key = self.get_key()
            
            if key == 'q':
                self.get_logger().info('Quitting keyboard control')
                break
            
            #根据按键设置目标速度，实际速度由S曲线平滑过渡
            if key == 'i' or key == 'w':
                self.target_linear = self.max_linear_speed
                self.target_angular = 0.0
            elif key == ',' or key == 's':
                self.target_linear = -self.max_linear_speed
                self.target_angular = 0.0
            elif key == 'j' or key == 'a':
                self.target_linear = 0.0
                self.target_angular = self.max_angular_speed
            elif key == 'l' or key == 'd':
                self.target_linear = 0.0
                self.target_angular = -self.max_angular_speed
            elif key == 'k':
                self.target_linear = 0.0
                self.target_angular = 0.0
            
            self.update_speed()
            
            cmd = Twist()
            cmd.linear.x = self.current_linear
            cmd.angular.z = self.current_angular
            self.cmd_vel_pub.publish(cmd)
            
            rclpy.spin_once(self, timeout_sec=self.dt)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
