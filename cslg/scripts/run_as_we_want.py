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
        rospy.init_node('RaWw', anonymous=True)

        # 创建发布者对象，发布到名为'cmd_vel'的主题，消息类型为Twist
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # 设置发布信息
        self.f = 20  # 频率 hz
        self.z = -0.2  # rad/s
        self.x = 0.1  # m/s

        # 预定路线路径
        self.routes_path = Path('routes.txt')

        # 预先声明计时起点
        self.t = time.perf_counter()

    def single_route(self, flag, x, tr, z):
        """
        实现单次行进
        :param flag: 转向还是行进标识
        :param z: 角速度，最高最好不要超过0.2
        :param x: 线速度，=角速度（rad）x半径（m）
        :param tr: 转弯总角度（四分之一角为1.6，平角为3.2，整圆6.3, 正左负右） 或 直行距离
        :return: None
        """
        pr = 0  # 当前行进路程

        # 当运行路程小于总路程时
        while abs(pr) <= abs(tr):
            if flag == 'c':
                pr += z * (1/self.f)
            elif flag == 's':
                pr += x * (1/self.f)
            else:
                pass
            tt = time.perf_counter() - self.t
            print(f'tt:{tt:.4f}s pr:{pr:.4f}rad x: {x:.4f}m/s z:{z:.4f}rad/s')

            # 创建速度消息对象
            vel_msg = Twist()

            # 设置线速度和角速度
            vel_msg.linear.x = x
            vel_msg.angular.z = z

            # 发布速度消息
            self.pub.publish(vel_msg)

            # 按照设定的频率休眠
            time.sleep(1/self.f)

    def route_master(self, routes_path):
        # 从路线文件读取所有路线
        with open(self.ROOT / routes_path, 'r') as fp:
            all_routes = fp.readlines()

        # 遍历路线运行
        for route in all_routes:
            flag, x, tr, z = route.split(',')
            z, tr, x = float(z), float(tr), float(x)
            self.single_route(flag, x, tr, z)


if __name__ == '__main__':
    try:
        dog_egg = RaWw()
        dog_egg.route_master(dog_egg.routes_path)
    except rospy.ROSInterruptException:
        pass

