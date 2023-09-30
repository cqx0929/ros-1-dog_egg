#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import rospy
import chardet
import numpy as np
from pathlib import Path
from sensor_msgs.msg import Image  # 导入图片消息类型


class ImageReceiver(object):
    def __init__(
            self,
            camera_topic,
            show_img,
            save_txt
    ):
        self.show_img = show_img
        self.save_txt = save_txt
        rospy.init_node('RosImageReceiver')
        self.img = None
        self.msg = None
        self.txt = './img_string.txt'
        rospy.Subscriber(camera_topic, Image, self.__img_callback)
        # depth_camera_pub = rospy.Publisher("/depth_camera", Image, queue_size=1)  # 创建一个名为depth_camera的话题发布器

    def receive_img(self):
        while not rospy.is_shutdown():
            self.__one_frame()

    def __img_callback(self, msg):
        self.msg = msg

    def __get_img(self):
        if self.msg:
            data = self.msg.data  # rgb8
            width = self.msg.width
            height = self.msg.height
            encoding = self.msg.encoding
            channel = int(len(data)/(height*width))
            data = [int(i) for i in data]
            # data = np.array(data).reshape(-1, 3)
            data = self.__get_flat_to_two_d(data, height, width, channel)
            self.img = data / 255
            # exit()
            # self.img_string = self.__get_txt(data)
            # self.__save_txt()
            # print('保存成功')

            # image = self.__get_flat_to_two_d(data, height, width, channel)
            # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # image = np.ndarray(shape=(height, width, channel), dtype=np.uint8, buffer=data)
            # self.img = np.zeros((height, width, channel), dtype=np.uint8)
            # self.img = image
        else:
            print('未收到图像信息！')

    def __save_txt(self):
        if self.txt is not None:
            with open(self.txt, 'w', encoding='utf-8') as fp:
                fp.write(self.img_string)
        else:
            print('文档保存失败')

    @staticmethod
    def __get_txt(data):
        s = ''
        for i in data:
            s += '{} '.format(i)
        return s

    @staticmethod
    def __get_flat_to_two_d(flat, height, width, channel):
        ls = [[[i for i in range(channel)] for j in range(width)] for k in range(height)]
        n = 0
        for y in range(height):
            for x in range(width):
                for z in range(channel-1, 0-1, -1):
                    ls[y][x][z] = flat[n]
                    n += 1
        return np.array(ls)

    def __show_img(self, i):
        if self.show_img:
            cv2.imshow("Image window", self.img)
            if cv2.waitKey(i) > 0:
                cv2.destroyAllWindows()

    def __one_frame(self):
        self.__get_img()
        if self.img is not None:
            self.__show_img(1)


if __name__ == '__main__':
    dg = ImageReceiver(
        camera_topic="/camera/rgb/image_raw",
        show_img=True,
        save_txt=True
    )
    dg.receive_img()
