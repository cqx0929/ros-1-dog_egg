#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName  :radar_manager_1.py
# @Time      :2023/9/11 13:00
# @Author    :CQX0929
import cv2
import math
import numpy as np


class RadarManager(object):
    def __init__(self):
        # 雷达二维图参数
        self.a = 800  # 图片边长
        self.img = np.zeros((self.a, self.a), dtype=np.uint8)  # 黑色的axa数组
        # self.radar_signal = np.ones(360, dtype=np.float64) * 6  # 模拟雷达信号
        self.radar_signal = np.random.uniform(0.0, 100.0, 5000)  # 模拟雷达信号
        self.radar_angle = 3.14  # 雷达角度范围
        self.radar_angle_sep = self.radar_angle / len(self.radar_signal)  # 雷达角度步长
        self.radar_distance = max(self.radar_signal)  # 雷达最大距离
        self.r = self.a/2  # 雷达圆盘半径
        self.r_sep = self.r / self.radar_distance  # 雷达长度和像素转换比例

    def axis(self):
        self.img[int(self.a/2), :] = 255  # 白色x轴
        self.img[:, int(self.a/2)] = 255  # 白色y轴
        np.fill_diagonal(self.img, 255)
        np.fill_diagonal(self.img[::-1], 255)

    def radar_data_to_cartesian_coordinate(self, radar_data):
        for i, data in enumerate(radar_data):
            theta = i * self.radar_angle_sep  # 角度
            rou = data * self.r_sep  # 长度
            # 横纵坐标
            x, y = (
                int(math.cos(theta) * rou + self.r),
                int(-math.sin(theta) * rou + self.r))
            cv2.circle(self.img, (x, y), 1, (255, 255, 255), thickness=cv2.FILLED)
        self.show()

    def show(self):
        cv2.imshow('img', self.img)
        if cv2.waitKey(1) > 0:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    gd = RadarManager()
    gd.axis()
    gd.radar_data_to_cartesian_coordinate(gd.radar_signal)


