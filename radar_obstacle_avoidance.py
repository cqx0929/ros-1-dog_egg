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

    def lidar_callback(self, data):
        self.radar_data = data

    def radar_obstacle_avoidance(self):
        radar_data = self.radar_data.ranges
        len_of_radar = len(radar_data)
        mid = radar_data[int(len_of_radar / 2 - 5):int(len_of_radar / 2 + 5)]
        side = radar_data[int(len_of_radar / 2 - 50):int(len_of_radar / 2 + 50)]
        left_avg = sum(side[:int(len(side)/2)]) / (len(side)/2)
        right_avg = sum(side[int(len(side)/2):]) / (len(side)/2)
        if sum(mid)/len(mid) < 0.5:
            if left_avg >= right_avg:
                self.dog_egg.single_wheel(0, 0.8, -0.8)
            else:
                self.dog_egg.single_wheel(0, 0.8, 0.8)
        else:
            self.dog_egg.single_straight(0.2, 0.0, 0)

    def roa_main(self):
        while not rospy.is_shutdown():
            if self.radar_data is not None:
                self.radar_obstacle_avoidance()


if __name__ == "__main__":
    gd = Roa()
    gd.roa_main()
