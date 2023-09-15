#! /usr/bin/env python3
# -*- coding:utf-8 -*-
# @FileName  :radar_manager_0.1.py
# @Time      :2023/9/11 13:00
# @Author    :CQX0929
import cv2
import numpy as np


class RadarManager(object):
    def __init__(self):
        self.radar_data = None
        # 雷达二维图参数
        self.a = 800  # 图片边长
        self.r = self.a / 2  # 雷达圆盘半径
        self.radar_angle = 3.14  # 雷达角度范围
        self.img = np.zeros((self.a, self.a), dtype=np.uint8)  # 黑色的axa数组
        self.radar_angle_sep = float()  # 雷达角度步长
        self.radar_distance = float()  # 雷达最大距离
        self.r_sep = float()  # 雷达长度和像素转换比例

    def radar_data_pre_process(self):
        self.radar_data = np.array(self.radar_data)
        for i in range(1, len(self.radar_data)-1):
            if self.radar_data[i] <= 0:
                self.radar_data[i] = (self.radar_data[i-1]*self.radar_data[i-1])**(1/2)

    def axis(self):
        self.img = np.zeros((self.a, self.a), dtype=np.uint8)  # 黑色的axa数组
        self.img[int(self.a/2), :] = 255  # 白色x轴
        self.img[:, int(self.a/2)] = 255  # 白色y轴
        np.fill_diagonal(self.img, 255)
        np.fill_diagonal(self.img[::-1], 255)
        cv2.circle(self.img, (int(self.r), int(self.r)), int(self.r), (255, 255, 255), thickness=1)

    def radar_data_to_cartesian_coordinate(self, index, distance):
        theta = index * self.radar_angle_sep  # 角度
        rou = distance * self.r_sep  # 长度
        x, y = int(np.cos(theta) * rou + self.r), int(-np.sin(theta) * rou + self.r)  # 横纵坐标
        return x, y

    def draw_radar_points(self, radar_data):
        for i in range(len(radar_data)):
            x, y = self.radar_data_to_cartesian_coordinate(i, radar_data[i])
            if x <= self.a and y <= self.a:
                cv2.circle(self.img, (int(x), int(y)), radius=1, color=(255, 255, 255), thickness=cv2.FILLED)

    def show(self, i):
        cv2.imshow('radar_data', self.img)
        if cv2.waitKey(i) > 0:
            cv2.destroyAllWindows()

    def one_frame(self):
        if self.radar_data is not None:
            # 获取并处理激光数据
            self.radar_data_pre_process()
            # 绘制坐标线
            self.axis()
            # 计算角度步长
            self.radar_angle_sep = self.radar_angle / len(self.radar_data)  # 雷达角度步长
            # 设定雷达最大距离
            # self.radar_distance = max(self.radar_data)  # 雷达最大距离
            self.radar_distance = 10.0
            # 雷达距离与像素之间的转换
            self.r_sep = self.r / self.radar_distance  # 雷达长度和像素转换比例
            # 绘图
            self.draw_radar_points(self.radar_data)
            # 展示
            self.show(1)
        else:
            print("等待激光数据...")

    def visualize_laser_data(self, data):
        self.radar_data = data
        self.one_frame()


if __name__ == "__main__":
    pass
