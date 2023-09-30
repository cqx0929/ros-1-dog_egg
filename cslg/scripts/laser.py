#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from run_as_we_want import RaWw


def LidarCallback(msg):
    global vel_pub
    dists = msg.ranges[110:251]
    left_dist = sum(dists[0:int(len(dists)/2)])/(len(dists)/2)
    right_dist = sum(dists[int(len(dists)/2):])/(len(dists)/2)
    mid_dist = msg.ranges[145:216]
    if sum(mid_dist)/len(mid_dist) < 0.8:
        if left_dist >= right_dist:
            gd.single_wheel(0, 0.8, -0.8)
        elif left_dist < right_dist:
            gd.single_wheel(0, 0.8, +0.8)
        else:
            pass
    else:
        gd.single_straight(0.2, 0.0, 0)


if __name__ == "__main__":
    gd = RaWw()
    lidar_sub = rospy.Subscriber("/scan", LaserScan, LidarCallback, queue_size=1)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.spin()
