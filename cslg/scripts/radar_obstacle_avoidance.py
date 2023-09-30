#! /usr/bin/env python3
# -*- coding:utf-8 -*-
# @FileName  :radar_obstacle_avoidance.py
# @Time      :2023/9/6 13:39
# @Author    :CQX0929


import rospy
from run_as_we_want import RaWw
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Roa(object):

    def __init__(self):
        self.dog_egg = RaWw()
        self.radar_data = None
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.oa_count = 0

    def lidar_callback(self, data):
        self.radar_data = data

    def radar_obstacle_avoidance(self):
        signal = [True, True, True]
        radar_data = self.radar_data.ranges
        len_of_radar = len(radar_data)
        mid = radar_data[int(len_of_radar / 2) - 5:int(len_of_radar / 2) + 5]
        left_mid = radar_data[int(len_of_radar / 2 - 20) - 11:int(len_of_radar / 2 - 20)]
        right_mid = radar_data[int(len_of_radar / 2 + 20):int(len_of_radar / 2 + 20) + 11]

        mid_avg = sum(mid) / len(mid)
        left_mid_avg = sum(left_mid) / len(left_mid)
        right_mid_avg = sum(right_mid) / len(right_mid)

        standard = 0.4  # 雷达阈值
        if mid_avg < standard:
            signal[1] = False
        if left_mid_avg < standard:
            signal[0] = False
        if right_mid_avg < standard:
            signal[2] = False

        side = radar_data[int(len_of_radar / 2 - 50):int(len_of_radar / 2 + 50)]
        left_side_avg = sum(side[:int(len(side)/2)]) / (len(side)/2)
        right_side_avg = sum(side[int(len(side)/2):]) / (len(side)/2)
        if self.oa_count < 0:
            self.oa_count = 0
        if signal == [True, True, True]:
            self.dog_egg.single_route('s', 0.2, 0.0, 0)
            self.oa_count -= 0.5
        elif (
            signal == [False, False, False]
            or self.oa_count > 3
        ):
            self.oa_count = 0
            self.dog_egg.single_route('c', 0, 3.14, 1.2)

        elif signal == [True, False, False]:
            self.dog_egg.single_route('c', -0.2, 0, -0.2)

        elif signal == [False, False, True]:
            self.dog_egg.single_route('c', -0.2, 0, 0.2)

        else:
            self.oa_count += 1
            if left_side_avg > right_side_avg:
                self.dog_egg.single_route('c', 0, 0.8, -0.8)
            else:
                self.dog_egg.single_route('c', 0, 0.8, 0.8)

    def roa_go(self):
        while not rospy.is_shutdown():
            if self.radar_data is not None:
                self.radar_obstacle_avoidance()


if __name__ == "__main__":
    gd = Roa()
    gd.roa_go()

