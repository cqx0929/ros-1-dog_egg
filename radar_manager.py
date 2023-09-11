#! /usr/bin/env python3
# -*- coding:utf-8 -*-
# @FileName  :radar_manager_0.1.py
# @Time      :2023/9/11 13:00
# @Author    :CQX0929


import cv2
import math
import time
import rospy
import scipy
import numpy as np
from sensor_msgs.msg import LaserScan


class RadarManager(object):
    def __init__(self):
        rospy.init_node('laser_data_visualizer', anonymous=True)  # ROS节点
        self.radar_data = None
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)  # 订阅激光雷达数据话题
        # 雷达二维图参数
        self.a = 400  # 图片边长
        self.r = self.a / 2  # 雷达圆盘半径
        self.radar_angle = 3.14  # 雷达角度范围
        self.img = np.zeros((self.a, self.a), dtype=np.uint8)  # 黑色的axa数组
        self.radar_angle_sep = float()  # 雷达角度步长
        self.radar_distance = float()  # 雷达最大距离
        self.r_sep = float()  # 雷达长度和像素转换比例

    def laser_callback(self, data):
        self.radar_data = data.ranges

    @staticmethod
    def radar_data_process(data):
        # 定义滤波器窗口大小
        window_size = 5

        # 移动平均滤波
        # window_size = 5
        # _data = np.convolve(data, np.ones(window_size) / window_size, mode='same')

        # 使用中值滤波
        _data = scipy.signal.medfilt(data, kernel_size=window_size)
        return _data

    def axis(self):
        self.img = np.zeros((self.a, self.a), dtype=np.uint8)  # 黑色的axa数组
        self.img[int(self.a/2), :] = 255  # 白色x轴
        self.img[:, int(self.a/2)] = 255  # 白色y轴
        np.fill_diagonal(self.img, 255)
        np.fill_diagonal(self.img[::-1], 255)
        cv2.circle(self.img, (int(self.r), int(self.r)), int(self.r), 255, thickness=1)

    def radar_data_to_cartesian_coordinate(self, radar_data):
        for i, data in enumerate(radar_data):
            theta = i * self.radar_angle_sep  # 角度
            rou = data * self.r_sep  # 长度
            # 横纵坐标
            x, y = (
                int(math.cos(theta) * rou + self.r),
                int(-math.sin(theta) * rou + self.r))
            if x <= self.a and y <= self.a:
                cv2.circle(self.img, (x, y), 1, 255, thickness=cv2.FILLED)
            else:
                pass

    def show(self, i):
        cv2.imshow('radar_data', self.img)
        if cv2.waitKey(i) > 0:
            cv2.destroyAllWindows()

    def visualize_laser_data(self):

        while not rospy.is_shutdown():
            t = time.perf_counter()
            if self.radar_data is not None:
                self.axis()
                self.radar_angle_sep = self.radar_angle / len(self.radar_data)  # 雷达角度步长
                # self.radar_distance = max(self.radar_data)  # 雷达最大距离
                self.radar_distance = 8.0
                self.r_sep = self.r / self.radar_distance  # 雷达长度和像素转换比例
                # 获取并处理激光数据
                radar_data = self.radar_data_process(self.radar_data)
                # 绘图
                self.radar_data_to_cartesian_coordinate(radar_data)
                # 展示
                self.show(1)
            else:
                print("等待激光数据...")
            fps = 1 / (time.perf_counter() - t)
            print(f'{fps:.2f}')


if __name__ == "__main__":
    gd = RadarManager()
    gd.visualize_laser_data()


