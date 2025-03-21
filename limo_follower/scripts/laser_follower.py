#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import time
import numpy as np
from collections import deque
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg
from limo_follower.msg import position as PositionMsg
from dynamic_reconfigure.server import Server
from limo_follower.cfg import laser_paramsConfig

class SimplePID:
    """简单的离散PID控制器"""
    def __init__(self, target, P, I, D):
        """
        创建一个离散PID控制器
        
        参数:
            target (double): 目标值
            P, I, D (double): PID参数
        """
        # 检查参数形状是否兼容
        if (not(np.size(P) == np.size(I) == np.size(D)) or 
            ((np.size(target) == 1) and np.size(P) != 1) or 
            (np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('输入参数形状不兼容')
            
        rospy.loginfo('PID初始化: P:{}, I:{}, D:{}'.format(P, I, D))
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.setPoint = np.array(target)
        
        self.last_error = 0
        self.integrator = 0
        self.integrator_max = float('inf')
        self.timeOfLastCall = None

    def update(self, current_value):
        """
        更新PID控制器
        
        参数:
            current_value (double): 当前值
            
        返回:
            控制信号 (double): 与目标长度相同的向量
        """
        current_value = np.array(current_value)  # [angleX, distance]
        
        if np.size(current_value) != np.size(self.setPoint):
            raise TypeError('当前值和目标值形状不一致')
            
        if self.timeOfLastCall is None:
            # PID第一次被调用，不知道时间差
            # 不应用控制信号
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))
        
        # 计算误差
        error = self.setPoint - current_value
        
        # 误差较小时停止移动
        if -0.1 < error[0] < 0.1:  # 角度误差
            error[0] = 0
        if -0.1 < error[1] < 0.1:  # 距离误差
            error[1] = 0
            
        # 当目标很小时，通过放大误差来提高速度
        if error[1] > 0 and self.setPoint[1] < 1.3:
            error[1] = error[1] * (1.3 / self.setPoint[1])
            
        P = error
        
        # 计算时间差
        currentTime = time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)
        
        # 误差的积分是当前误差*时间差
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        
        # 误差的微分是误差的差值/时间差
        D = (error - self.last_error) / deltaT
        
        self.last_error = error
        self.timeOfLastCall = currentTime
        
        # 返回控制信号
        return self.Kp * P + self.Ki * I + self.Kd * D

class VelocityFilter:
    """速度滤波器，用于平滑速度命令"""
    def __init__(self, filter_size=5, alpha=0.3):
        """
        初始化滤波器
        
        参数:
            filter_size (int): 移动平均滤波器窗口大小
            alpha (float): 低通滤波器系数 (0-1)，越小滤波效果越强
        """
        self.filter_size = filter_size
        self.alpha = alpha
        self.linear_history = deque(maxlen=filter_size)
        self.angular_history = deque(maxlen=filter_size)
        self.last_linear = 0.0
        self.last_angular = 0.0
        
    def moving_average_filter(self, value, history):
        """移动平均滤波"""
        history.append(value)
        return sum(history) / len(history)
    
    def low_pass_filter(self, new_value, last_value):
        """低通滤波"""
        return self.alpha * new_value + (1 - self.alpha) * last_value
    
    def filter(self, linear_vel, angular_vel):
        """
        对速度进行滤波
        
        参数:
            linear_vel (float): 线速度
            angular_vel (float): 角速度
            
        返回:
            (float, float): 滤波后的线速度和角速度
        """
        # 先进行移动平均滤波
        linear_avg = self.moving_average_filter(linear_vel, self.linear_history)
        angular_avg = self.moving_average_filter(angular_vel, self.angular_history)
        
        # 再进行低通滤波
        filtered_linear = self.low_pass_filter(linear_avg, self.last_linear)
        filtered_angular = self.low_pass_filter(angular_avg, self.last_angular)
        
        # 更新上一次的值
        self.last_linear = filtered_linear
        self.last_angular = filtered_angular
        
        return filtered_linear, filtered_angular

class LaserFollower:
    def __init__(self):
        # 初始化参数
        # 移除controllerLossTimer，避免1秒后自动停止
        
        # 从参数服务器获取参数
        self.max_speed = rospy.get_param('~maxSpeed', 0.8)
        self.targetDist = rospy.get_param('~targetDist', 0.3)
        self.maxTrackingDistance = rospy.get_param('~maxTrackingDistance', 8.0)  # 最大跟踪距离，默认5米
        self.publish_rate = rospy.get_param('~publish_rate', 10)  # 发布频率，默认10Hz
        self.filter_size = rospy.get_param('~filter_size', 8)  # 滤波器窗口大小
        self.filter_alpha = rospy.get_param('~filter_alpha', 0.1)  # 低通滤波器系数
        self.angle_deadzone = rospy.get_param('~angle_deadzone', 0.25)  # 角度死区，弧度
        self.distance_deadzone = rospy.get_param('~distance_deadzone', 0.25)  # 距离死区，米
        
        # PID参数
        self.PID_param = {
            'P': [0.8, 0.8],  # [线速度P, 角速度P]
            'I': [0.0, 0.0],  # [线速度I, 角速度I]
            'D': [0.1, 0.1]   # [线速度D, 角速度D]
        }
        
        # 状态变量
        self.active = True
        self.last_valid_position = None
        self.position_timeout = rospy.Duration(3.0)  # 增加超时时间到3秒
        self.last_position_time = rospy.Time.now()
        self.last_cmd_vel_time = rospy.Time.now()
        
        # 创建速度滤波器
        self.velocity_filter = VelocityFilter(self.filter_size, self.filter_alpha)
        
        # 动态参数配置服务器
        self.dynamic_reconfigure_server = Server(laser_paramsConfig, self.reconfigCB)
        
        # 创建PID控制器
        self.PID_controller = SimplePID([0, self.targetDist], 
                                        self.PID_param['P'], 
                                        self.PID_param['I'], 
                                        self.PID_param['D'])
        
        # 发布速度命令 - 控制limo2移动
        self.cmdVelPublisher = rospy.Publisher('limo2/cmd_vel', Twist, queue_size=10)
        
        # 订阅目标位置和跟踪器信息
        self.positionSubscriber = rospy.Subscriber('/object_tracker/current_position', 
                                                  PositionMsg, 
                                                  self.positionUpdateCallback)
        self.trackerInfoSubscriber = rospy.Subscriber('/object_tracker/info', 
                                                     StringMsg, 
                                                     self.trackerInfoCallback)
        
        # 创建定时器，定期检查位置更新和发布速度命令
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)
        
        # 创建速度命令发布定时器，确保持续发布速度命令
        self.cmd_vel_timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publishCmdVel)
        
        # 当前速度命令
        self.current_cmd_vel = Twist()
        
        # 当程序终止时停止移动
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("激光雷达跟随器已初始化，使用limo2的激光雷达数据跟踪最近物体")
        rospy.loginfo("最大跟踪距离: %.2f 米，发布频率: %d Hz", self.maxTrackingDistance, self.publish_rate)
        rospy.loginfo("滤波器参数: 窗口大小=%d, 低通系数=%.2f", self.filter_size, self.filter_alpha)
        rospy.loginfo("死区设置: 角度=%.3f 弧度, 距离=%.3f 米", self.angle_deadzone, self.distance_deadzone)

    def reconfigCB(self, config, level):
        """动态参数配置回调函数"""
        self.max_speed = config.maxSpeed
        self.targetDist = config.targetDist
        self.maxTrackingDistance = config.maxTrackingDistance
        self.angle_deadzone = config.angle_deadzone
        self.distance_deadzone = config.distance_deadzone
        self.filter_alpha = config.filter_alpha
        
        # 更新滤波器参数
        self.velocity_filter.alpha = self.filter_alpha
        
        # 更新PID参数
        self.PID_param['P'] = [config.P_v, config.P_w]
        self.PID_param['I'] = [config.I_v, config.I_w]
        self.PID_param['D'] = [config.D_v, config.D_w]
        
        # 重新创建PID控制器
        self.PID_controller = SimplePID([0, self.targetDist], 
                                        self.PID_param['P'], 
                                        self.PID_param['I'], 
                                        self.PID_param['D'])
        
        rospy.loginfo("更新参数: max_speed=%.2f, targetDist=%.2f, maxTrackingDistance=%.2f", 
                     self.max_speed, self.targetDist, self.maxTrackingDistance)
        rospy.loginfo("更新死区: angle_deadzone=%.3f, distance_deadzone=%.3f", 
                     self.angle_deadzone, self.distance_deadzone)
        rospy.loginfo("更新滤波器: filter_alpha=%.2f", self.filter_alpha)
        return config

    def trackerInfoCallback(self, info):
        """处理跟踪器信息"""
        # 记录信息，如果是"nothing found"类型的消息，可能需要停止移动
        rospy.logwarn(info.data)

    def timerCallback(self, event):
        """定时器回调函数，检查位置更新是否超时"""
        if self.active and (rospy.Time.now() - self.last_position_time) > self.position_timeout:
            rospy.logwarn("位置更新超时 (%.1f 秒)，停止移动", (rospy.Time.now() - self.last_position_time).to_sec())
            self.stopMoving()

    def publishCmdVel(self, event=None):
        """定期发布速度命令"""
        if self.active:
            # 检查上次速度命令发布时间，如果超过一定时间没有更新，则停止移动
            if (rospy.Time.now() - self.last_cmd_vel_time).to_sec() > 0.5:  # 如果0.5秒内没有新的速度命令，则保持当前速度
                pass  # 继续发布当前速度命令
                
            # 发布当前速度命令
            self.cmdVelPublisher.publish(self.current_cmd_vel)
            rospy.logdebug("发布速度命令: 线速度=%.2f, 角速度=%.2f", 
                          self.current_cmd_vel.linear.x, self.current_cmd_vel.angular.z)

    def apply_deadzone(self, value, deadzone):
        """应用死区"""
        if -deadzone < value < deadzone:
            return 0.0
        elif value > 0:
            return value - deadzone
        else:
            return value + deadzone

    def positionUpdateCallback(self, position):
        """处理目标位置更新"""
        if not self.active:
            return  # 如果不活跃，立即返回
        
        # 更新最后位置时间
        self.last_position_time = rospy.Time.now()
        
        # 获取目标角度和距离
        angleX = position.angleX
        distance = position.distance
        
        # 检查距离是否在最大跟踪距离内
        if distance > self.maxTrackingDistance:
            rospy.logwarn("目标距离 %.2f 米超出最大跟踪距离 %.2f 米，停止跟踪", distance, self.maxTrackingDistance)
            self.stopMoving()
            return
        
        # 保存有效位置
        self.last_valid_position = position
        
        # 调整角度，使其在-π到π之间
        if angleX > 0:
            angleX = angleX - 3.1415
        else:
            angleX = angleX + 3.1415
        
        # 应用死区
        angleX = self.apply_deadzone(angleX, self.angle_deadzone)
        distance_error = distance - self.targetDist
        distance_error = self.apply_deadzone(distance_error, self.distance_deadzone)
        
        # 如果两个误差都在死区内，停止移动
        if angleX == 0 and distance_error == 0:
            rospy.loginfo("目标在死区内，停止移动")
            self.stopMoving()
            return
        
        # 调用PID控制器更新并获取新的速度
        [uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])
        
        # 将速度限制在最大速度范围内
        angularSpeed = np.clip(uncliped_ang_speed, -self.max_speed, self.max_speed)
        linearSpeed = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)
        
        # 应用速度滤波
        filtered_linear, filtered_angular = self.velocity_filter.filter(linearSpeed, angularSpeed)
        
        # 更新当前速度命令
        self.current_cmd_vel = Twist()
        self.current_cmd_vel.linear = Vector3(filtered_linear, 0, 0)  # 线速度
        self.current_cmd_vel.angular = Vector3(0, 0, filtered_angular)  # 角速度
        
        # 更新最后速度命令时间
        self.last_cmd_vel_time = rospy.Time.now()
        
        # 立即发布速度命令
        self.cmdVelPublisher.publish(self.current_cmd_vel)
        
        # 调试信息
        rospy.loginfo("控制: 角度=%.2f, 距离=%.2f, 原始速度=[%.2f, %.2f], 滤波后=[%.2f, %.2f]", 
                     angleX, distance, linearSpeed, angularSpeed, filtered_linear, filtered_angular)

    def stopMoving(self):
        """停止移动"""
        self.current_cmd_vel = Twist()
        self.current_cmd_vel.linear = Vector3(0, 0, 0)
        self.current_cmd_vel.angular = Vector3(0, 0, 0)
        self.cmdVelPublisher.publish(self.current_cmd_vel)
        rospy.loginfo("停止移动")

    def shutdown(self):
        """关闭节点时的清理工作"""
        # 停止移动
        self.stopMoving()
        self.active = False
        rospy.loginfo("激光雷达跟随器已关闭")

if __name__ == '__main__':
    rospy.init_node('laser_follower')
    
    # 设置日志级别
    rospy.loginfo("设置日志级别为INFO")
    
    # 创建跟随器对象
    follower = LaserFollower()
    rospy.loginfo('激光雷达跟随器已启动')
    
    # 设置循环频率
    rate = rospy.Rate(20)  # 20Hz
    
    try:
        # 使用循环而不是rospy.spin()，确保节点持续运行
        while not rospy.is_shutdown():
            # 在循环中可以添加额外的处理逻辑
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo('激光雷达跟随器已停止')
    finally:
        # 确保在退出时停止移动
        follower.shutdown()
