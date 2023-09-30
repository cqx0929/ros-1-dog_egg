#! /usr/bin/env python
# -*-coding:utf8 -*-
from turtlesim.msg import Pose
import rospy
"""
   需求：订阅并输出乌龟的位姿消息
   1.导包
   2.初始化ros节点
   3.创建订阅对象
   4.使用回调函数处理订阅信息
   5.spin()
"""
def dopose(pose):
   rospy.loginfo("->乌龟消息为`：坐标（%.2f,%.2f)\n朝向:%.2f\nlinear:%.2f\nangular:%.2f",
                 pose.x,pose.y,pose.theta,pose.linear_velocity,pose.angular_velocity)
if __name__ == "__main__":
   # 初始化ros节点
   rospy.init_node("yubanwangluo7")
   # 创建订阅对象
   sub = rospy.Subscriber("/turtle1/pose",Pose,dopose,queue_size=100)
   # 使用回调函数处理订阅的消息
   # spin（）
   rospy.spin()
