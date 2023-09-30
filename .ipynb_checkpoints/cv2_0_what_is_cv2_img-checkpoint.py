#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName  :cv2_0_what_is_cv2_img.py
# @Time      :2023/9/19 15:42
# @Author    :CQX0929
import cv2
import numpy as np

img = cv2.imread('img/lena.png', cv2.IMREAD_COLOR)
print(img[0][0])
print(img[0])
img = np.ndarray(shape=(512, 512, 3), buffer=img)
print(img[0][0])
print(img[0])

# img = img[0]/255
print(img.shape)
cv2.imshow('img', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
