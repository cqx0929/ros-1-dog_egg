#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName  :cv2_1_moment_test.py
# @Time      :2023/9/14 20:32
# @Author    :CQX0929


import cv2
import numpy as np


def camera():
    cap = cv2.VideoCapture(0)
    while True:
        # 读取摄像头帧
        ret, frame = cap.read()

        if not ret:
            break

        # 在窗口中显示帧
        frame = contour_area_filter(frame)
        cv2.imshow('Camera', frame)

        # 检测按键 'q'，如果按下则退出循环
        if cv2.waitKey(1) > 0:
            break

    # 释放摄像头资源
    cap.release()

    # 关闭所有打开的窗口
    cv2.destroyAllWindows()


def contour_area_filter(image):
    # 读取图像
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 175, 255, cv2.THRESH_BINARY)[1]
    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 1)
    thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

    # 计算图像的矩
    for cnt in contours:
        moments = cv2.moments(cnt)
        # 从矩中提取所需的特征
        area = moments['m00']  # 图像的面积
        if int(area) not in range(20 ** 2, 512 ** 2):
            continue

        centroid_x = moments['m10'] / area  # X 方向的质心坐标
        centroid_y = moments['m01'] / area  # Y 方向的质心坐标

        c_x = int(centroid_x)
        c_y = int(centroid_y)
        a = int(area**(1/2))
        # print("x: {}, y: {}, size: {}".format(c_x, c_y, area))
        cv2.circle(image, (c_x, c_y), 1, (0, 0, 255), cv2.FILLED)
        cv2.rectangle(image, (c_x - a, c_y - a), (c_x + a, c_y + a), (0, 0, 255), 1)
    return np.hstack((image, thresh))


if __name__ == '__main__':
    camera()
