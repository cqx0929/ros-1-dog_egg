#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName  :socket-1-request.py
# @Time      :2023/9/15 13:02
# @Author    :CQX0929
import socket
import numpy as np
from radar_manager_1 import RadarManager


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
        while True:
            # 接收服务器的响应
            data = self.client_socket.recv(10240)
            if len(data) == 0:
                break
            data = eval(data.decode())
            self.axis()
            self.radar_data_to_cartesian_coordinate(data)
            self.show()
            # self.radar_data_to_cartesian_coordinate(np.array(tuple(data.decode())))

        # 关闭连接
        self.client_socket.close()


if __name__ == "__main__":
    gd = SocketRequest()
    gd.run()
