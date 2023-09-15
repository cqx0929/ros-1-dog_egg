#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName  :socket_1_request.py
# @Time      :2023/9/15 13:02
# @Author    :CQX0929
import time
import socket
from radar_manager import RadarManager


class SocketRequest(RadarManager):
    def __init__(self):
        RadarManager.__init__(self)

        # 服务器配置
        server_host = '192.168.0.195'  # 请替换为Linux服务器的IP地址或主机名
        server_port = 12345

        # 创建套接字
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 连接到服务器
        self.client_socket.connect((server_host, server_port))

    def run(self):
        fps = 0
        t = time.perf_counter()
        while True:
            # 接收服务器的响应
            radar_data = self.client_socket.recv(2000)
            if len(radar_data) == 0:
                break
            radar_data = radar_data.decode().split(' ')
            radar_data = [float(i) for i in radar_data if i != '']
            self.visualize_laser_data(radar_data)
            fps += 1
            if time.perf_counter() - t > 1:
                print(fps)
                fps = 0
                t = time.perf_counter()

        # 关闭连接
        self.client_socket.close()


if __name__ == "__main__":
    gd = SocketRequest()
    gd.run()
