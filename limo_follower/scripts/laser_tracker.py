#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String as StringMsg
from limo_follower.msg import position as PositionMsg
from dynamic_reconfigure.server import Server
from limo_follower.cfg import laser_paramsConfig

class LaserTracker:
    def __init__(self):
        # 初始化参数
        self.lastScan = None
        
        # 从参数服务器获取参数
        self.winSize = rospy.get_param('~winSize', 5)
        self.deltaDist = rospy.get_param('~deltaDist', 0.2)
        self.maxTrackingDistance = rospy.get_param('~maxTrackingDistance', 5.0)  # 最大跟踪距离，默认5米
        
        # 订阅激光雷达数据 - 修改为使用limo2的激光雷达
        self.scanSubscriber = rospy.Subscriber('limo2/scan', LaserScan, self.registerScan)
        
        # 发布目标位置和信息
        self.positionPublisher = rospy.Publisher('object_tracker/current_position', PositionMsg, queue_size=3)
        self.infoPublisher = rospy.Publisher('object_tracker/info', StringMsg, queue_size=3)
        
        # 动态参数配置服务器
        self.dynamic_reconfigure_server = Server(laser_paramsConfig, self.reconfigCB)
        
        rospy.loginfo("激光雷达跟踪器已初始化，使用limo2的激光雷达数据，最大跟踪距离: %.2f 米", self.maxTrackingDistance)

    def reconfigCB(self, config, level):
        """动态参数配置回调函数"""
        self.winSize = config.winSize
        self.deltaDist = config.deltaDist
        self.maxTrackingDistance = config.maxTrackingDistance
        rospy.loginfo("更新参数: winSize=%d, deltaDist=%.2f, maxTrackingDistance=%.2f", 
                     self.winSize, self.deltaDist, self.maxTrackingDistance)
        return config

    def registerScan(self, scan_data):
        """处理激光扫描数据并发布最近物体的位置"""
        # 将激光扫描数据转换为numpy数组
        ranges = np.array(scan_data.ranges)
        
        # 过滤掉超过最大跟踪距离的点
        filtered_ranges = np.copy(ranges)
        filtered_ranges[filtered_ranges > self.maxTrackingDistance] = np.inf
        
        # 按距离排序，从近到远检查点是否是真实物体
        sortedIndices = np.argsort(filtered_ranges)
        
        minDistanceID = None
        minDistance = float('inf')
        
        if self.lastScan is not None:
            # 如果已有上一次扫描数据进行比较
            for i in sortedIndices:
                # 从最近的点开始检查
                tempMinDistance = filtered_ranges[i]
                
                # 如果距离已经是无穷大，说明没有更多有效点了
                if np.isinf(tempMinDistance):
                    break
                
                # 检查是否是噪声：
                # 在上一次扫描的窗口内查找是否有相似距离的点
                
                # 裁剪窗口以避免索引越界
                windowIndex = np.clip([i-self.winSize, i+self.winSize+1], 0, len(self.lastScan))
                window = self.lastScan[windowIndex[0]:windowIndex[1]]
                
                with np.errstate(invalid='ignore'):
                    # 检查窗口内是否有距离接近当前点的扫描
                    if np.any(abs(window-tempMinDistance) <= self.deltaDist):
                        # 找到一个合理的距离
                        minDistanceID = i
                        minDistance = ranges[minDistanceID]
                        break  # 至少有一个点距离相近，找到有效最小值，结束循环
        
        # 保存当前扫描数据用于下次比较
        self.lastScan = ranges
        
        # 如果没有扫描到物体或最小距离超出范围
        if minDistance > self.maxTrackingDistance or np.isinf(minDistance):
            # 发布警告，未找到物体或物体超出跟踪范围
            rospy.logwarn('激光雷达未找到5米内的物体')
            self.infoPublisher.publish(StringMsg('laser:nothing found within range'))
        else:
            # 计算物体位置的角度，0度为正前方
            minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
            
            # 发布物体位置信息
            self.positionPublisher.publish(PositionMsg(minDistanceAngle, 0, minDistance))
            
            # 调试信息
            rospy.loginfo("找到物体: 角度=%.2f, 距离=%.2f", minDistanceAngle, minDistance)

if __name__ == '__main__':
    rospy.init_node('laser_tracker')
    tracker = LaserTracker()
    rospy.loginfo('激光雷达跟踪器已启动')
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('激光雷达跟踪器已停止')
