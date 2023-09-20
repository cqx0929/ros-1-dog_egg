#! /usr/bin/env python3
# -*- coding:utf-8 -*-
# @FileName  :radar_2_radar_manager_ros.py
# @Time      :2023/9/11 13:00
# @Author    :CQX0929


import cv2
import time
import rospy
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
        self.radar_data = np.array(data.ranges)

    @staticmethod
    def radar_data_average_complement(data):
        data = np.array(data)
        # for i in range(1, len(data)-1):
        #     if data[i] <= 0.001:
        #         data[i] = (data[i-1] * data[i+1]) ** (1/2)
        return data[data > 0]

    def axis(self):
        self.img = np.zeros((self.a, self.a), dtype=np.uint8)  # 黑色的axa数组
        self.img[int(self.a/2), :] = 255  # 白色x轴
        self.img[:, int(self.a/2)] = 255  # 白色y轴
        np.fill_diagonal(self.img, 255)
        np.fill_diagonal(self.img[::-1], 255)
        cv2.circle(self.img, (int(self.r), int(self.r)), int(self.r), (255, 255, 255), thickness=1)

    def kalman(self, data):
        # 初始化状态估计
        x_estimated = np.array([0.0])  # 初始状态估计
        P_estimated = np.array([1.0])  # 初始状态估计的协方差

        # 系统模型参数
        A = np.array([1.0])  # 状态转移矩阵
        B = np.array([0.0])  # 控制输入矩阵
        H = np.array([1.0])  # 观测矩阵
        Q = np.array([0.1])  # 系统过程噪声协方差
        R = np.array([1.0])  # 观测噪声协方差

        # 观测数据
        observations = data

        # 预测步骤（时间更新）
        x_predicted = A * x_estimated + B * 0  # 控制输入为0
        P_predicted = A * P_estimated * A + Q

        # 更新步骤（测量更新）
        K = P_predicted * H / (H * P_predicted * H + R)
        x_estimated = x_predicted + K * (observations - H * x_predicted)
        P_estimated = (1 - K * H) * P_predicted
        return x_estimated

    def radar_data_to_cartesian_coordinate(self, index, distance):
        theta = index * self.radar_angle_sep  # 角度
        rou = distance * self.r_sep  # 长度
        x, y = int(np.cos(theta) * rou + self.r), int(-np.sin(theta) * rou + self.r)  # 横纵坐标
        return x, y

    def draw_points(self, radar_data):
        # xs, ys = [], []
        for i in range(len(radar_data)):
            x, y = self.radar_data_to_cartesian_coordinate(i, radar_data[i])
        #     xs.append(x)
        #     ys.append(y)
        # ys = self.kema(ys)
        # for x, y in zip(xs, ys):
            if x <= self.a and y <= self.a:
                cv2.circle(self.img, (int(x), int(y)), radius=1, color=(255, 255, 255), thickness=cv2.FILLED)

    def show(self, i):
        cv2.imshow('radar_data', self.img)
        if cv2.waitKey(i) > 0:
            cv2.destroyAllWindows()

    def one_frame(self):
        if self.radar_data is not None:
            self.axis()
            self.radar_angle_sep = self.radar_angle / len(self.radar_data[self.radar_data > 0])  # 雷达角度步长
            # self.radar_distance = max(self.radar_data)  # 雷达最大距离
            self.radar_distance = 8.0
            self.r_sep = self.r / self.radar_distance  # 雷达长度和像素转换比例
            # 获取并处理激光数据
            radar_data = self.radar_data_average_complement(self.radar_data)
            # 绘图
            self.draw_points(radar_data)
            # 展示
            self.show(1)
        else:
            print("等待激光数据...")

    def visualize_laser_data(self):

        while not rospy.is_shutdown():
            t = time.perf_counter()
            self.one_frame()
            fps = 1 / (time.perf_counter() - t)
            print(f'{fps:.2f}')


if __name__ == "__main__":
    gd = RadarManager()
    gd.visualize_laser_data()

