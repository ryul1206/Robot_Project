#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
# Topic
import nav_msgs.msg
import geometry_msgs.msg


class Broadcaster:
    def __init__(self):
        self.sub_pose = rospy.Subscriber(
            '/robot/true_odom', nav_msgs.msg.Odometry, self.callback_odom)
        self.sub_arm = rospy.Subscriber(
            '/robot/arm_end', geometry_msgs.msg.Pose, self.callback_arm)
        self.robot_pose = [0.0, 0.0, 0.0]
        self.arm_info = [[0.17, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0]]

        self.br = tf.TransformBroadcaster()
        self.info = {
            'TF_SCAN_FW': [
                (0.21447, 0.0, 0.11886),
                (0.0, 0.0, 0.0, 1.0),
                'TF_BASE'],
            'TF_ARM': [
                (0.049526, 0.0, 0.37103),
                (0.0, 0.0, 0.0, 1.0),
                'TF_BASE'],
            'TF_CAM': [
                (0.11972, 0.0, 0.059851),
                (0.0, 0.0, 0.0, 1.0),
                'TF_END'],
            'TF_WORLD': [
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                'map']
        }

    def callback_odom(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.robot_pose[2] = euler[2]

    def callback_arm(self, msg):
        self.arm_info[0][0] = msg.position.x
        self.arm_info[0][1] = msg.position.y
        self.arm_info[0][2] = msg.position.z
        self.arm_info[1][0] = msg.orientation.x
        self.arm_info[1][1] = msg.orientation.y
        self.arm_info[1][2] = msg.orientation.z
        self.arm_info[1][3] = msg.orientation.w

    def main(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            # rospy.logwarn('%f'%(now.to_nsec()))
            # Fixed tf
            for child in self.info:
                self.br.sendTransform(
                    self.info[child][0],
                    self.info[child][1],
                    now, child, self.info[child][2])

            # Dynamic tf
            self.br.sendTransform(
                (self.robot_pose[0], self.robot_pose[1], 0.0),
                tf.transformations.quaternion_from_euler(0.0, 0.0, self.robot_pose[2]),
                now, 'TF_BASE', 'TF_WORLD')
            self.br.sendTransform(
                self.arm_info[0],
                self.arm_info[1],
                now, 'TF_END', 'TF_ARM')
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('tf_broadcaster')
    try:
        m = Broadcaster()
        m.main()
    except rospy.ROSInterruptException:
        pass
