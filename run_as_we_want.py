#! /usr/bin/env python3
# -*- coding:utf-8 -*-
# @FileName  :run_as_we_want.py
# @Time      :2023/9/5 15:45
# @Author    :CQX0929

import os
import sys
import time
import rospy
from pathlib import Path
from geometry_msgs.msg import Twist


class RaWw(object):

    def __init__(self):
        # 获取根目录路径
        FILE = Path(__file__).resolve()
        ROOT = FILE.parents[0]  # root directory
        if str(ROOT) not in sys.path:
            sys.path.append(str(ROOT))  # add ROOT to PATH
        self.ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

        # 初始化节点
        # rospy.init_node('velocity_publisher', anonymous=True)

        # 创建发布者对象，发布到名为'cmd_vel'的主题，消息类型为Twist
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # 设置发布信息
        self.f = 20  # 频率 hz
        self.z = -0.2  # rad/s
        self.x = 0.1  # m/s

        # self.tz = 1.6  # 圆周运动目标角度 target z rad

        # 预定路线路径
        self.routes_path = Path('routes.txt')

        # 预先声明计时起点
        self.t = None

    def single_wheel(self, x, tz, z):
        """
        实现单次转弯
        :param z: 转弯角速度，最高最好不要超过0.2
        :param x: 转弯线速度，=角速度（rad）x半径（m）
        :param tz: 转弯总角度，四分之一角为1.6，平角为3.2，整圆6.3, 正左负右
        :return: None
        """
        az = 0  # 单次圆周运动总角度 all z rad

        # 当运行角度小于总角度时
        while abs(az) <= abs(tz):
            az += z * (1/self.f)
            tt = time.perf_counter() - self.t
            print(f'tt:{tt:.4f}s az:{az:.4f}rad z:{z:.4f}rad/s')

            # 创建速度消息对象
            vel_msg = Twist()

            # 设置线速度和角速度
            vel_msg.linear.x = x
            vel_msg.angular.z = z

            # 发布速度消息
            self.pub.publish(vel_msg)

            # 按照设定的频率休眠
            time.sleep(1/self.f)

    def single_straight(self, x, tx, z):
        """
        实现单次直行
        :param z: 直行角速度，为 0 rad/s
        :param x: 直行线速度，0.1 ~ 2 m/s
        :param tx: 直行距离
        :return: None
        """
        ax = 0
        while abs(ax) <= abs(tx):
            ax += x * (1/self.f)
            tt = time.perf_counter() - self.t
            print(f'tt:{tt:.4f}s ax:{ax:.4f}m x:{x:.4f}m/s')

            # 创建速度消息对象
            vel_msg = Twist()

            # 设置线速度和角速度
            vel_msg.linear.x = x
            vel_msg.angular.z = z

            # 发布速度消息
            self.pub.publish(vel_msg)

            # 按照设定的频率休眠
            time.sleep(1/self.f)

    def route_master(self):
        # 从路线文件读取所有路线
        with open(self.ROOT / self.routes_path, 'r') as fp:
            all_routes = fp.readlines()
        # 遍历路线运行
        self.t = time.perf_counter()
        for route in all_routes:
            flag, x, tz, z = route.split(',')
            z, tz, x = float(z), float(tz), float(x)
            if flag == 'c':
                self.single_wheel(x, tz, z)
            elif flag == 's':
                self.single_straight(x, tz, z)
            else:
                pass


if __name__ == '__main__':
    try:
        dog_egg = RaWw()
        dog_egg.route_master()
    except rospy.ROSInterruptException:
        pass
