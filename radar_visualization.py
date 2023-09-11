#! /usr/bin/env python3
# -*- coding:utf-8 -*-
# @FileName  :radar_visualization.py
# @Time      :2023/9/6 12:55
# @Author    :CQX0929

import time
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LaserDataVisualizer:
    def __init__(self):
        rospy.init_node('laser_data_visualizer', anonymous=True)
        self.laser_data = None

        # 订阅激光雷达数据话题
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def laser_callback(self, data):
        self.laser_data = data

    def visualize_laser_data(self):
        plt.ion()  # 打开Matplotlib的交互式模式
        fig, ax = plt.subplots()

        while not rospy.is_shutdown():
            if self.laser_data is not None:
                # 获取激光数据
                ranges = self.laser_data.ranges

                # 绘制激光雷达数据
                ax.clear()
                ax.plot(ranges)
                ax.set_title('Laser Scan Data')
                ax.set_xlabel('Measurement Index')
                ax.set_ylabel('Distance (m)')
                ax.set_ylim(0, max(ranges) + 1)

                plt.pause(0.1)  # 更新图形，可以调整更新频率
            else:
                rospy.loginfo("等待激光数据...")


if __name__ == '__main__':
    try:
        visualizer = LaserDataVisualizer()
        visualizer.visualize_laser_data()
    except rospy.ROSInterruptException:
        pass