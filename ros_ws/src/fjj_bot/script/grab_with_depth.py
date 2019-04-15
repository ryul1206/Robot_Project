#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import copy
import numpy as np
import rospy
import sensor_msgs.msg
import fjj_bot.msg


def get_angle_dist_pair(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dist = np.sqrt(dx**2.0 + dy**2.0)
    if dx == 0.0:
        dx = 0.00000001
    min_angle = np.arctan(dy/dx)
    return (dist, min_angle)


class DepthGrab:
    def __init__(self):
        # Subscribe
        self.img_sub = rospy.Subscriber("/image/hand_depth",
                                        sensor_msgs.msg.Image,
                                        self.callback_func,
                                        queue_size=1)
        self.h = 480
        self.w = 640
        self.cen = (self.w/2.0, self.h/2.0)
        self.cv_image = np.zeros((self.h, self.w, 3), np.uint8)
        # Publish
        self.grab_pub = rospy.Publisher(
            'robot/grab_info', fjj_bot.msg.TargetInfo, queue_size=10)
        self.info = fjj_bot.msg.TargetInfo()

    def callback_func(self, rosimg):
        temp = np.frombuffer(rosimg.data, dtype=np.uint8).reshape(rosimg.height, rosimg.width, -1)
        temp = cv2.cvtColor(temp, cv2.COLOR_BGR2RGB)
        self.cv_image = cv2.flip(temp, 0)

    def main(self):
        rate = rospy.Rate(32)
        ratio = np.deg2rad(57.0)/self.w  # rad/pixel
        # min_len, x_pixel, y_pixel, dtheta_z
        buffer = [(0.0, 0.0, 0.0, 0.0)]
        while not rospy.is_shutdown():
            gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            gray_image = cv2.Canny(gray_image, 10, 20)

            # gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, binary_image = cv2.threshold(
                gray_image, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            # _,thresh = cv2.threshold(binary_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            _, contours, hierarchy = cv2.findContours(
                binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            try:
                cnt = contours[0]
                x, y, w, h = cv2.boundingRect(cnt)
                self.cv_image = cv2.rectangle(
                    self.cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                self.cv_image = cv2.drawContours(
                    self.cv_image, [box], 0, (0, 0, 255), 2)
                # Publish
                x_pixel = int((box[0][0]+box[1][0]+box[2][0]+box[3][0])/4.0)
                y_pixel = int((box[0][1]+box[1][1]+box[2][1]+box[3][1])/4.0)
        
                # dtheta_z
                candidate = [get_angle_dist_pair(box[0], box[1]),
                             get_angle_dist_pair(box[0], box[2]),
                             get_angle_dist_pair(box[0], box[3])]
                candidate.sort()
                dtheta_z = candidate[0][1]  # dtheta_z

                if str(rospy.get_param('/state')) == 'catching':
                    buffer.append((candidate[0][0], x_pixel, y_pixel, dtheta_z))
                if len(buffer) > 5:
                    buffer.sort()
                    del buffer[0]

                # self.cv_image = cv2.line(
                #     self.cv_image, tuple(box[0]), tuple(box[1]), (255, 0, 0), 3)
                # self.cv_image = cv2.line(
                #     self.cv_image, tuple(box[0]), tuple(box[2]), (0, 100, 200), 3)
                # self.cv_image = cv2.line(
                #     self.cv_image, tuple(box[0]), tuple(box[3]), (0, 255, 0), 3)
            except IndexError as e:
                pass

            # if rospy.get_param('/state') == 'catching':
            # rospy.logfatal
            # print(type(rospy.get_param('/state')))
            buffer.sort()
            largest = len(buffer)-1
            x_pix = buffer[largest][1]
            y_pix = buffer[largest][2]
            dtheta_z = buffer[largest][3]

            dtheta_x = (x_pix-self.cen[0])*ratio  # dtheta_x
            dtheta_y = (self.cen[1]-y_pix)*ratio  # dtheta_y

            self.info.dtheta_x = dtheta_x
            self.info.dtheta_y = dtheta_y
            self.info.dtheta_z = dtheta_z

            center = (int(self.cen[0]+x_pix), int(self.cen[1]+y_pix))
            self.cv_image = cv2.circle(
                self.cv_image, center, 10, (0, 255, 0), -1)
            dx = int(100 * np.cos(self.info.dtheta_z))
            dy = int(100 * np.sin(self.info.dtheta_z))
            p1 = (int(center[0] + dx), int(center[1] + dy))
            p2 = (int(center[0] - dx), int(center[1] - dy))
            self.cv_image = cv2.line(
                self.cv_image, p1, p2, (255, 255, 0), 3)
            self.cv_image = cv2.circle(
                self.cv_image, (int(x_pix), int(y_pix)), 10, (255, 255, 0), -1)


            self.grab_pub.publish(copy.deepcopy(self.info))

            cv2.imshow('Depth Vision', self.cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('grab_with_depth')
    try:
        grab = DepthGrab()
        grab.main()
    except rospy.ROSInterruptException:
        pass
