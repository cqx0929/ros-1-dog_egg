#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
"""
     发布方实现 发布速度消息
          话题 /turtlel/cmd_vel
          消息 geometry_msgs/Twist
    1.导包
    2,初始化ros节点
    3,创建发布者对象
    4,组织数据并发布数据
"""
if __name__ == "__main__":
    # 初始化ros节点
    rospy.init_node("yubanwangluo88")
    # 创建发布者对象
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist()
    
    twist.linear.x = 0.2
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.5
    rospy.loginfo("已启动")
    # 循环发布
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()
