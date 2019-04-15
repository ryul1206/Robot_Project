#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import matplotlib.pyplot as plt
from darkflow.net.build import TFNet

import rospy
import sensor_msgs.msg
import fjj_bot.msg


DARKNET_PATH = '/home/hr/sw/darkflow/'
options = {
    'model': DARKNET_PATH+'cfg/yolov2.cfg',
    'load': DARKNET_PATH+'bin/yolov2.weights',
    'labels': '/home/hr/sw/fjj_team_project/'+'labels.txt',
    'threshold': 0.45,
    'gpu': 1.0
}

interest_label = [
    'bottle',
    'toy car',
    'bucket',
    'sports ball'
]


class Yolo:
    def __init__(self):
        self.tfnet = TFNet(options)
        # Subscribe
        self.img_sub = rospy.Subscriber("/image/hand_color",
                                        sensor_msgs.msg.Image,
                                        self.callback_func,
                                        queue_size=1)
        self.h = 480
        self.w = 640
        self.cen = (self.w/2.0, self.h/2.0)
        self.cv_image = np.zeros((self.h, self.w, 3), np.uint8)
        # Publish
        self.target_pub = rospy.Publisher(
            'robot/target_info', fjj_bot.msg.TargetInfo, queue_size=10)
        self.white_list = []  # {label, pixel, count, time_first}
        self.target_idx = -1

    def callback_func(self, rosimg):
        temp = np.frombuffer(rosimg.data, dtype=np.uint8).reshape(rosimg.height, rosimg.width, -1)
        temp = cv2.cvtColor(temp, cv2.COLOR_BGR2RGB)
        self.cv_image = cv2.flip(temp, 0)

    def publish_target_info(self, detect, name, x_pixel, y_pixel):
        info = fjj_bot.msg.TargetInfo()
        info.detection = detect
        info.name = name
        ratio = np.deg2rad(57.0)/self.w  # rad/pixel
        info.dtheta_x = (x_pixel-self.cen[0])*ratio
        info.dtheta_y = (self.cen[1]-y_pixel)*ratio
        self.target_pub.publish(info)

    def focusing(self, results):
        threshold = 20
        if rospy.get_param('/state') != 'focusing':
            if self.white_list:
                self.publish_target_info(False, '', 0.0, 0.0)
                del self.white_list[:]
                self.target_idx = -1
        else:
            for r in results:
                if r['label'] in interest_label:
                    p = ((r['topleft']['x']+r['bottomright']['x'])/2.0,
                        (r['topleft']['y']+r['bottomright']['y'])/2.0)
                    idx = -1
                    min_dist = 9999
                    for i in range(len(self.white_list)):
                        comp = self.white_list[i]
                        if r['label'] == comp['label']:
                            dpixel = (p[0]-comp['pixel'][0], p[1]-comp['pixel'][1])
                            dist = abs(dpixel[0]/1.5) + abs(dpixel[1])
                            if (dist < 200) and (dist < min_dist):
                                min_dist = dist
                                idx = i
                    if idx < 0:
                        candidate = {'label': r['label'],
                                    'pixel': p,
                                    'count': 0,
                                    'time_first': rospy.get_time()}
                        self.white_list.append(candidate)
                    else:
                        old_p = self.white_list[idx]['pixel']
                        self.white_list[idx]['pixel'] = ((p[0]*0.8+old_p[0]*0.2),
                                                        (p[1]*0.8+old_p[1]*0.2))
                        self.white_list[idx]['count'] += 1
                        if (self.white_list[idx]['count'] > threshold) and (self.target_idx < 0):
                            self.target_idx = idx
        if not (self.target_idx < 0):
            focus = self.white_list[self.target_idx]
            result_idx = -1
            min_dist = 9999
            for k in range(len(results)):
                temp = results[k]
                if temp['label'] ==  focus['label']:
                    p = ((temp['topleft']['x']+temp['bottomright']['x'])/2.0,
                         (temp['topleft']['y']+temp['bottomright']['y'])/2.0)
                    
                    dpixel = (p[0]-focus['pixel'][0], p[1]-focus['pixel'][1])
                    dist = abs(dpixel[0]/1.5) + abs(dpixel[1])
                    if (dist < 200) and (dist < min_dist):
                        min_dist = dist
                        result_idx = k
            self.publish_target_info(
                (False if result_idx < 0 else True),
                focus['label'],
                focus['pixel'][0],
                focus['pixel'][1])

    def main(self):
        rate = rospy.Rate(32)
        colors = [tuple(255 * np.random.rand(3)) for _ in range(30)]
        while not rospy.is_shutdown():
            results = self.tfnet.return_predict(self.cv_image)

            self.focusing(results)

            for color, result in zip(colors, results):
                tl = (result['topleft']['x'], result['topleft']['y'])
                br = (result['bottomright']['x'], result['bottomright']['y'])
                label = result['label']
                conf = result['confidence']
                text = '{}: {:.0f}%'.format(label, conf * 100)
                self.cv_image = cv2.rectangle(
                    self.cv_image, tl, br, color, 3)
                self.cv_image = cv2.rectangle(
                    self.cv_image, tl, (tl[0]+13*len(text), tl[1]+20), color, -1)
                self.cv_image = cv2.putText(
                    self.cv_image, text, (tl[0], tl[1]+10), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 0), 2)
            for w in self.white_list:
                p = (int(w['pixel'][0]), int(w['pixel'][1]))
                self.cv_image = cv2.circle(
                    self.cv_image, p, 10, (0,0,255), 2)
                self.cv_image = cv2.putText(
                    self.cv_image, str(w['count'])+':'+w['label'], p,
                    cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 0), 2)
            if self.target_idx > 0:
                w = self.white_list[self.target_idx]
                p = (int(w['pixel'][0]), int(w['pixel'][1]))
                self.cv_image = cv2.circle(
                    self.cv_image, p, 6, (0,255,0), -1)
            cv2.imshow('RGB Vision', self.cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('yolo')
    try:
        darknet = Yolo()
        darknet.main()
    except rospy.ROSInterruptException:
        pass
