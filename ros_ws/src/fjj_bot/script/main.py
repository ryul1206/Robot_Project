#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import tf
import rospy
import copy
from vrepsim import VrepSimulation
# Message
import std_msgs.msg
# from geometry_msgs.msg import Pose2D
import nav_msgs.msg
import geometry_msgs.msg
import fjj_bot.msg


def dist(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


class Main:
    def __init__(self):
        self.vrep = VrepSimulation()

        # tf
        self.listener = tf.TransformListener()

        # Publish
        self.pub_hand = rospy.Publisher(
            'robot/closing', std_msgs.msg.Bool, latch=True, queue_size=1)
        self.pub_hand.publish(False)

        # Subscribe
        self.target_info = fjj_bot.msg.TargetInfo()
        self.sub_target = rospy.Subscriber(
            'robot/target_info', fjj_bot.msg.TargetInfo, self.callback_target, queue_size=10)
        self.grab_info = fjj_bot.msg.TargetInfo()
        self.sub_grab = rospy.Subscriber(
            'robot/grab_info', fjj_bot.msg.TargetInfo, self.callback_grab, queue_size=10)

        # Publish(temp)
        self.pub_true_odom = rospy.Publisher(
            'robot/true_odom', nav_msgs.msg.Odometry, queue_size=50)
        # rospy.Subscriber('robot/pos', RobotPos, self.cb_pos)

    def publish_true_odom(self, pose, quat, linear, angular):
        true_odom = nav_msgs.msg.Odometry()
        true_odom.header.stamp = rospy.Time.now()
        true_odom.header.frame_id = "TF_WORLD"
        true_odom.child_frame_id = "TF_BASE"
        true_odom.pose.pose.position = \
            geometry_msgs.msg.Point(pose[0], pose[1], pose[2])
        true_odom.pose.pose.orientation = \
            geometry_msgs.msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
        # true_odom.pose.covariance
        true_odom.twist.twist.linear = \
            geometry_msgs.msg.Vector3(linear[0], linear[1], linear[2])
        true_odom.twist.twist.angular = \
            geometry_msgs.msg.Vector3(angular[0], angular[1], angular[2])
        # true_odom.twist.covariance
        self.pub_true_odom.publish(true_odom)

    def callback_target(self, msg):
        self.target_info = msg

    def callback_grab(self, msg):
        self.grab_info = msg

    def localization(self):
        (pose, quat, linear, angular) = self.vrep.get_true_robot_odom()
        self.publish_true_odom(pose, quat, linear, angular)

    def main(self):
        if self.vrep.is_not_ready:
            raise NotImplementedError

        self.localization()

        rospy.sleep(1)

        while not rospy.is_shutdown():
            rospy.set_param('/state', 'step_init_pose')
            self.step_init_pose()

            rospy.set_param('/state', 'focusing')
            self.step_find_target()
            self.step_goto_target()

            rospy.set_param('/state', 'catching')
            self.step_catch_object()

            rospy.set_param('/state', 'find_path')
            self.step_find_path()

            rospy.sleep(10000)

        # rate = rospy.Rate(32)
        # while not rospy.is_shutdown():
        #     ###########
        #     z = 0.33947 # + np.sin(now)/10.0
        #     self.vrep.set_end_effector(
        #         (0.16699, 0.0, z),
        #         tf.transformations.quaternion_from_euler(0.0, np.pi/6.0, 0.0))
        #     ###########
        #     rate.sleep()

    def step_init_pose(self):
        self.localization()
        self.vrep.set_wheel_vel(0, 0)
        self.move_TF_END_with_rock(
            (0.17, 0.0, 0.3), tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0))

    def step_find_target(self):
        self.move_TF_END_with_rock(
            (0.15, 0.0, 0.3), tf.transformations.quaternion_from_euler(0.0, np.pi/6.0, 0.0))
        tstart = rospy.get_time()
        while (not self.target_info.detection) and (not rospy.is_shutdown()):
            self.localization()
            rospy.sleep(0.02)
            dt = rospy.get_time()-tstart
            rospy.loginfo(dt)
            rate = dt*(2.0*np.pi)/10.0  # 10sec
            self.vrep.set_end_effector(
                (0.15, 0.0, 0.3), tf.transformations.quaternion_from_euler(0.0, np.pi/6.0, np.sin(rate)))

    def step_goto_target(self):
        reach = False
        end_gain = [0.15, 0.000001]  # P, I
        end_error = {'x':0.0, 'y':0.0}
        end_euler_new = [0.0, 0.0, 0.0]
        bot_gain = [3.0, 2.0]  # P_r, P_theta
        bot_error = {'r':0.0, 'theta':0.0}

        while (not reach) and (not rospy.is_shutdown()):
            self.localization()
            # arm end
            end_pose, end_quat = self.vrep.get_end_effector()
            end_euler_old = tf.transformations.euler_from_quaternion(end_quat)
            end_euler_new[0] = 0.0
            end_euler_new[1] = end_euler_old[1] - (self.target_info.dtheta_y*end_gain[0]) - end_error['y']
            end_euler_new[2] = end_euler_old[2] - (self.target_info.dtheta_x*end_gain[0]) - end_error['x']
            end_error['x'] += self.target_info.dtheta_x*end_gain[1]
            end_error['y'] += self.target_info.dtheta_y*end_gain[1]
            # body
            down_angle = end_euler_old[1] - self.target_info.dtheta_y
            end_z = end_pose[2]
            bot_error['r'] = (end_z / np.tan(down_angle))  # estimated_r
            bot_error['theta'] = end_euler_old[2] - self.target_info.dtheta_x
            linear_vel = min(bot_error['r'] * bot_gain[0], 3.0)
            angular_vel = bot_error['theta'] * bot_gain[1]
            self.vrep.set_wheel_vel(linear_vel - angular_vel,
                                    linear_vel + angular_vel)

            # set
            n_quat = tf.transformations.quaternion_from_euler(
                end_euler_new[0], end_euler_new[1], end_euler_new[2])
            mody = tf.transformations.euler_from_quaternion(n_quat)
            quat = tf.transformations.quaternion_from_euler(
                mody[0], mody[1], mody[2])
            self.vrep.set_end_effector((0.15, 0.0, 0.3), quat)
            rospy.loginfo(
                'end_err[x]:%f, [y]:%f / bot_err[r]:%f, [th]:%fdeg'
                % (end_error['x'], end_error['y'], bot_error['r'],
                   np.rad2deg(bot_error['theta'])))
            
            if bot_error['r'] < 0.1:
                self.vrep.set_wheel_vel(0.0, 0.0)
                reach = True
            rospy.sleep(0.01)

    def step_catch_object(self):
        ready = False
        end_gain = 0.15
        # end_error = {'x':0.0, 'y':0.0}
        end_euler_new = [0.0, 0.0, 0.0]
        bot_gain = [3.0, 2.0]  # P_r, P_theta
        bot_error = {'r':0.0, 'theta':0.0}

        # snapshot = copy.deepcopy(self.grab_info)
        end_pose, end_quat = self.vrep.get_end_effector()
        end_euler_old = tf.transformations.euler_from_quaternion(end_quat)
        mody = self.grab_info.dtheta_z
        if mody > np.deg2rad(20):
            mody = np.deg2rad(20)
        elif mody < np.deg2rad(-20):
            mody = np.deg2rad(-20)
        truth_x = end_euler_old[0] + mody
        # down_angle = end_euler_old[1] - self.grab_info.dtheta_y
        dz = 0.003
        # x_shift = 0.07
        # extra = 0
        while (not ready) and (not rospy.is_shutdown()):
            # rospy.logfatal('1')
            self.localization()
            # rospy.logfatal('2')
            # arm end
            end_pose, end_quat = self.vrep.get_end_effector()
            # rospy.logfatal('2')
            end_euler_old = tf.transformations.euler_from_quaternion(end_quat)
            # rospy.logfatal('4')
            end_euler_new[0] = end_euler_old[0] + (truth_x - end_euler_old[0])*end_gain
            end_euler_new[1] = end_euler_old[1]
            end_euler_new[2] = end_euler_old[2]
            # end_error['x'] += self.grab_info.dtheta_x*end_gain[1]
            # end_error['y'] += self.grab_info.dtheta_y*end_gain[1]
            new_pose = (end_pose[0] + dz*np.tan((np.pi/2.0)-end_euler_new[1]+np.deg2rad(3)),
                        end_pose[1],
                        end_pose[2] - dz)
            # if extra < x_shift:
            #     extra += dz
            #     new_pose = (end_pose[0] + dz + dz*np.tan((np.pi/2.0)-end_euler_new[1]),
            #                 end_pose[1],
            #                 end_pose[2] - dz)
            # else:
            # set
            # rospy.logfatal('5')
            n_quat = tf.transformations.quaternion_from_euler(
                end_euler_new[0], end_euler_new[1], end_euler_new[2])
            mody = tf.transformations.euler_from_quaternion(n_quat)
            quat = tf.transformations.quaternion_from_euler(
                mody[0], mody[1], mody[2])
            # rospy.logfatal('6')
            self.vrep.set_end_effector(new_pose, quat)
            # rospy.logfatal('7')
            sensor_dist = self.vrep.get_hand_dist_vec()

            rospy.loginfo(
                'pose[x]:%f, [z]:%f, [sensor]:%f' % (new_pose[0], new_pose[2], sensor_dist[2]))

            # if new_pose[2] < 0.3:
            #     rospy.set_param('/state', 'auto')
            # rospy.logfatal('8')
            if (sensor_dist[2] < 0.02) and (sensor_dist[0]==0.0) and (sensor_dist[1]==0.0):  # mm
                rospy.logfatal('out')
                ready = True
            # rospy.logfatal('9')
            rospy.sleep(0.01)
        
        self.pub_hand.publish(True)
        rospy.sleep(5)

    def step_find_path(self):
        self.localization()
        self.step_init_pose()
        pass

    #############
    # euler is zrot -> yrot -> xrot (not fixed angle)
    def move_TF_END_with_rock(self, target_pose, target_quat,
                                 dist_speed=0.2, ang_speed=0.3):
        s_pose, s_quat = self.vrep.get_end_effector()
        strech_ratio = 0.5 / np.sqrt(target_pose[0]**2 + target_pose[1]**2)
        if strech_ratio < 1.0:
            target_pose = (target_pose[0]*strech_ratio,
                           target_pose[1]*strech_ratio,
                           target_pose[2]*strech_ratio)
        # Angular Coordinate: r, th, z
        s_ang = self.pose_to_angular(s_pose)
        g_ang = self.pose_to_angular(target_pose)
        diff_ang = (g_ang[0] - s_ang[0],
                    g_ang[1] - s_ang[1],
                    g_ang[2] - s_ang[2])
        diff_quat = (target_quat[0] - s_quat[0],
                     target_quat[1] - s_quat[1],
                     target_quat[2] - s_quat[2],
                     target_quat[3] - s_quat[3])

        # dt dist
        r_th = diff_ang[1] * (s_ang[0]+g_ang[0])/2.0
        diff_time_dist = np.sqrt(r_th**2 + diff_ang[2]**2) / dist_speed
        # dt ang
        quat_sqsum = 0.0
        for i in diff_quat:
            quat_sqsum += (i**2)
        diff_time_ang = np.sqrt(quat_sqsum) / ang_speed
        dt_max = max(diff_time_dist, diff_time_ang)

        # rospy.logwarn('start x: %f, y:%f, z:%f'%(s_pose[0], s_pose[1], s_pose[2]))
        # rospy.logwarn('targe x: %f, y:%f, z:%f'%(target_pose[0], target_pose[1], target_pose[2]))
        s_time = rospy.get_time()
        while not rospy.is_shutdown():
            ratio = (rospy.get_time() - s_time) / dt_max
            if ratio < 0.98:
                n_ang = (s_ang[0] + (diff_ang[0] * ratio),
                         s_ang[1] + (diff_ang[1] * ratio),
                         s_ang[2] + (diff_ang[2] * ratio))
                n_pose = self.angular_to_pose(n_ang)
                n_quat = (s_quat[0] + (diff_quat[0] * ratio),
                          s_quat[1] + (diff_quat[1] * ratio),
                          s_quat[2] + (diff_quat[2] * ratio),
                          s_quat[3] + (diff_quat[3] * ratio))
                e = tf.transformations.euler_from_quaternion(n_quat)
                mody = tf.transformations.quaternion_from_euler(e[0], e[1], e[2])
                self.vrep.set_end_effector(n_pose, mody)
            else:
                self.vrep.set_end_effector(target_pose, target_quat)
                break
            rospy.sleep(0.01)
        return (False if strech_ratio < 1.0 else True)

    def pose_to_angular(self, pose):
        r = np.sqrt(pose[0]**2 + pose[1]**2)
        th = np.arctan2(pose[1], pose[0])
        z = pose[2]
        return (r, th, z)

    def angular_to_pose(self, angular):
        x = angular[0]*np.cos(angular[1])
        y = angular[0]*np.sin(angular[1])
        z = angular[2]
        return (x, y , z)



if __name__ == "__main__":
    rospy.init_node('main')
    try:
        m = Main()
        m.main()
    except rospy.ROSInterruptException:
        pass
