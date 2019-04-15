#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from rosgraph_msgs.msg import Clock


class CustomClock():
    def __init__(self):
        self.sub = rospy.Subscriber(
            '/simulationTime', Float32, self.callback_clock)
        self.pub = rospy.Publisher(
            '/clock', Clock, queue_size=10)

    def callback_clock(self, msg):
        self.pub.publish(rospy.Time(msg.data))


if __name__ == "__main__":
    rospy.init_node('rosClock')
    rospy.set_param('/use_sim_time', True)
    try:
        c = CustomClock()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
