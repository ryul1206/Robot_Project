# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 21:33:10 2018

@author: woosiggi
"""
import numpy as np
import cv2


img = cv2.imread('grim1.png',1)
gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
_ , binary_image = cv2.threshold(gray_image,127,255,cv2.THRESH_BINARY)

_,thresh = cv2.threshold(binary_image,127,255,cv2.THRESH_BINARY)

_,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

try:
        cnt = contours[0]
except IndexError as e:
        errimg = binary_image

x,y,w,h = cv2.boundingRect(cnt)
img= cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

rect = cv2.minAreaRect(cnt)
box = cv2.boxPoints(rect)
box = np.int0(box)
img = cv2.drawContours(img,[box],0,(0,0,255),2)



cv2.imshow('ret',img)
cv2.waitKey(0)
cv2.destroyAllWindows()



##cv2.imshow('binary_image',binary_image)
##cv2.waitKey(0)
##cv2.destroyAllWindows()
