#!/usr/bin/env python
# -*- coding:utf-8 -*-
import sys
import rospy
import numpy as np
from sensor_msgs.msg import Image  # 导入深度相机话题的消息类型
import cv2
from cv_bridge import CvBridge, CvBridgeError


def depth_camera_callback(msg):
    try:
        # 使用cv_bridge将ROS消息转换为OpenCV图像格式
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", str(e))
        return

    draw_img = depth_image.copy()
    colormap = cv2.applyColorMap(draw_img, cv2.COLORMAP_JET)
    cvImg = cv2.cvtColor(colormap, 6)
    npImg = np.asarray(cvImg)
    #thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]

        # 检测目标物体的轮廓
    findcon_img, cnts, hierarchy = cv2.findContours(npImg, cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    for c in cnts:
            # 把轮廓描绘出来，并绘制中心点
            cv2.drawContours(findcon_img, [c], -1, (0, 0, 255), 2)
    # 在这里添加你的深度图像处理代码
    # canny
    # findcounts
    # max(area)
    # if a< average(max(x)) < b
    # get(x,y)
    # turn
    # elif average < a
    # back
    # else
    # run
    # 这里只是一个示例，你可以根据你的需求进行深度图像处理
    # 发布处理后的深度图像到名为depth_camera的话题
    depth_image_msg = bridge.cv2_to_imgmsg(findcon_img, encoding="passthrough")
    depth_camera_pub.publish(depth_image_msg)
 
if __name__ == '__main__':
    rospy.init_node('depth_camera_subscriber')

    # 订阅深度相机话题
    depth_camera_topic = "/camera/depth/image_raw"  # 替换为实际的深度相机话题
    rospy.Subscriber(depth_camera_topic, Image, depth_camera_callback)

    # 创建一个名为depth_camera的话题发布器
    depth_camera_pub = rospy.Publisher("/depth_camera", Image, queue_size=1)

    rospy.spin()

