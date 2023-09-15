#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# @FileName  :socket_2_server.py
# @Time      :2023/9/15 15:27
# @Author    :CQX0929
import cv2
import rospy
import socket
import numpy as np
from sensor_msgs.msg import LaserScan


class RadarServer(object):

    def __init__(self):
        self.data = None
        rospy.init_node('socket_ros_node', anonymous=True)

        # 创建Socket服务器
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 12345))  # 绑定到所有网络接口的指定端口
        self.server_socket.listen(1)
        print("正在等待客户端连接！")
        self.client_socket, self.client_address = self.server_socket.accept()
        print("已接受来自客户端的连接")

        # 订阅ROS话题
        rospy.Subscriber('/scan', LaserScan, self.callback)

    def run(self):
        while not rospy.is_shutdown():
            try:
                # 等待客户端断开连接
                if self.client_socket.fileno() == -1:
                    break
            except Exception:
                self.server_socket.close()
                breakd

    def callback(self, data):
        s = ''
        self.data = data.ranges
        for i, data in enumerate(self.data):
            data = round(data, 2)
            s += f'{data} '
        print(len(s))
        if len(self.data) > 0:
            self.client_socket.send(s.encode())


if __name__ == '__main__':
    try:
        gd = RadarServer()
        gd.run()
    except rospy.ROSInterruptException:
        pass
