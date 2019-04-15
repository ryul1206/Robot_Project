#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import vrep_lib.vrep as vrep
import geometry_msgs.msg

TIME_OUT = 4.0


class VrepSimulation():
    client_id = -1
    handles = {}
    names = ['TF_WORLD',  # pretected
             'TF_BASE',  # pretected
             'TF_ARM',   # pretected
             'TF_END',   # 'redundantRob_manipSphere',
             'Pioneer_p3dx_leftMotor',
             'Pioneer_p3dx_rightMotor',
             'BarrettHand_Sensor']

    def __init__(self):
        rospy.loginfo('Wait until /use_sim_time==True')
        while not rospy.is_shutdown():
            if rospy.get_param('/use_sim_time'):
                break
        rospy.loginfo('Now /use_sim_time==True')
        # Connect to V-REP
        # close all opened connections
        vrep.simxFinish(-1)
        ip, port = ('127.0.0.1', 19997)
        self.is_not_ready = True
        # commThreadCycleInMs is 32ms
        self.client_id = vrep.simxStart(ip, port, True, True, 5000, 32)
        if self.client_id == -1:
            rospy.logerr('vrepsim: Connection was failed.')
            self.pause_simulation()
        elif not self.init_handles():
            rospy.logerr('vrepsim: init handles failed.')
            self.pause_simulation()
        else:
            self.is_not_ready = False
        # Pub
        self.pub_TF_ARM = rospy.Publisher(
            '/robot/arm_end', geometry_msgs.msg.Pose, queue_size=10)

    ###############
    def get_true_robot_odom(self):
        err = vrep.simx_return_remote_error_flag
        while err != vrep.simx_return_ok:
            err, pose = vrep.simxGetObjectPosition(
                self.client_id, self.handles['TF_BASE'], self.handles['TF_WORLD'],
                vrep.simx_opmode_oneshot)
        err = vrep.simx_return_remote_error_flag
        while err != vrep.simx_return_ok:
            err, quat = vrep.simxGetObjectQuaternion(
                self.client_id, self.handles['TF_BASE'], self.handles['TF_WORLD'],
                vrep.simx_opmode_oneshot)
        err = vrep.simx_return_remote_error_flag
        while err != vrep.simx_return_ok:
            err, linear, angular = vrep.simxGetObjectVelocity(
                self.client_id, self.handles['TF_BASE'],
                vrep.simx_opmode_oneshot)
        return (pose, quat, linear, angular)

    def get_end_effector(self):
        err = vrep.simx_return_remote_error_flag
        while err != vrep.simx_return_ok:
            err, pose = vrep.simxGetObjectPosition(
                self.client_id, self.handles['TF_END'], self.handles['TF_ARM'],
                vrep.simx_opmode_oneshot)
        err = vrep.simx_return_remote_error_flag
        while err != vrep.simx_return_ok:
            err, quat = vrep.simxGetObjectQuaternion(
                self.client_id, self.handles['TF_END'], self.handles['TF_ARM'],
                vrep.simx_opmode_oneshot)
        # rospy.logfatal('     x: %f, y:%f, z:%f'%(pose[0], pose[1], pose[2]))
        return (pose, quat)
    
    def get_hand_dist_vec(self):
        err = vrep.simx_return_remote_error_flag
        while err != vrep.simx_return_ok:
            err, _state, _p, _h, _normal_vector = vrep.simxReadProximitySensor(
                self.client_id, self.handles['BarrettHand_Sensor'], vrep.simx_opmode_oneshot)
        rospy.loginfo('%f, %f, %f'%(_p[0], _p[1], _p[2]))
        return _p

    def set_end_effector(self, pose, quat):
        err = vrep.simx_return_remote_error_flag
        while err != vrep.simx_return_ok:
            err = vrep.simxSetObjectPosition(
                self.client_id, self.handles['TF_END'], self.handles['TF_ARM'],
                pose, vrep.simx_opmode_oneshot)
        err = vrep.simx_return_remote_error_flag
        while err != vrep.simx_return_ok:
            err = vrep.simxSetObjectQuaternion(
                self.client_id, self.handles['TF_END'], self.handles['TF_ARM'],
                quat, vrep.simx_opmode_oneshot)
        self.get_end_effector()
        end = geometry_msgs.msg.Pose()
        end.position.x = pose[0]
        end.position.y = pose[1]
        end.position.z = pose[2]
        end.orientation.x = quat[0]
        end.orientation.y = quat[1]
        end.orientation.z = quat[2]
        end.orientation.w = quat[3]
        self.pub_TF_ARM.publish(end)

    def set_wheel_vel(self, left_vel, right_vel):
        max_vel = 3.0
        vrep.simxPauseCommunication(self.client_id, True)
        vrep.simxSetJointTargetVelocity(
            self.client_id, self.handles['Pioneer_p3dx_leftMotor'],
            min(left_vel, max_vel), vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(
            self.client_id, self.handles['Pioneer_p3dx_rightMotor'],
            min(right_vel, max_vel), vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(self.client_id, False)

    ###############
    def init_handles(self):
        last_name = ''
        try:
            for n in self.names:
                last_name = n
                self.handles[n] = self.get_handle(n)
        except RuntimeError as e:
            rospy.logerr('There is no name: '+last_name)
            return False
        return True

    def get_handle(self, name):
        err = vrep.simx_return_timeout_flag
        t = time.time()
        handle = 0
        while err != vrep.simx_return_ok:
            err, handle = vrep.simxGetObjectHandle(
                self.client_id, name, vrep.simx_opmode_oneshot_wait)
            if (time.time() - t) > TIME_OUT:
                raise RuntimeError
        return handle

    ###############
    def start_simulation(self):
        try:
            self.send_until_ok(vrep.simxStartSimulation)
            rospy.logwarn("vrep started")
        except RuntimeError as e:
            rospy.logerr("vrep failed to start: %s" % e)

    def pause_simulation(self):
        try:
            self.send_until_ok(vrep.simxPauseSimulation)
            rospy.logwarn("vrep paused")
        except RuntimeError as e:
            rospy.logerr("vrep failed to pause: %s" % e)

    def stop_simulation(self):
        try:
            self.send_until_ok(vrep.simxStopSimulation)
            rospy.logwarn("vrep stopped")
        except RuntimeError as e:
            rospy.logerr("vrep failed to stop: %s" % e)

    def send_until_ok(self, func):
        err = vrep.simx_return_timeout_flag
        t = time.time()
        while err != vrep.simx_return_ok:
            err = func(self.client_id, vrep.simx_opmode_oneshot)
            if (time.time() - t) > TIME_OUT:
                raise RuntimeError


    # def cb_pos(self, data):
    #     # position
    #     vrep.simxSetObjectPosition(self.client_id,
    #                                self.rb_hd[data.rid],
    #                                self.origin_hd,
    #                                (data.pos.x, data.pos.y, 0),
    #                                vrep.simx_opmode_oneshot)
    #     # wheel
    #     dx = data.pos.x - self.prev_pos[data.rid][0]
    #     dy = data.pos.y - self.prev_pos[data.rid][1]
    #     if dx > 0:
    #         if dy > 0:
    #             self.wh_rot(data.rid, 0, -self.vel, 0, self.vel)
    #         elif dy < 0:
    #             self.wh_rot(data.rid, -self.vel, 0, self.vel, 0)
    #         else:
    #             self.wh_rot(data.rid, -self.vel, -self.vel, self.vel, self.vel)
    #     elif dx < 0:
    #         if dy > 0:
    #             self.wh_rot(data.rid, self.vel, 0, -self.vel, 0)
    #         elif dy < 0:
    #             self.wh_rot(data.rid, 0, self.vel, 0, -self.vel)
    #         else:
    #             self.wh_rot(data.rid, self.vel, self.vel, -self.vel, -self.vel)
    #     else:
    #         if dy > 0:
    #             self.wh_rot(data.rid, self.vel, -self.vel, -self.vel, self.vel)
    #         elif dy < 0:
    #             self.wh_rot(data.rid, -self.vel, self.vel, self.vel, -self.vel)
    #         else:
    #             self.wh_rot(data.rid, 0, 0, 0, 0)
    #     self.prev_pos[data.rid][0] = data.pos.x
    #     self.prev_pos[data.rid][1] = data.pos.y

    # def wh_rot(self, rid, a, b, c, d):
    #     i = rid * 4
    #     vrep.simxSetJointTargetVelocity(self.client_id, self.wh_hd[i],
    #                                     a, vrep.simx_opmode_oneshot)
    #     vrep.simxSetJointTargetVelocity(self.client_id, self.wh_hd[i + 1],
    #                                     b, vrep.simx_opmode_oneshot)
    #     vrep.simxSetJointTargetVelocity(self.client_id, self.wh_hd[i + 2],
    #                                     c, vrep.simx_opmode_oneshot)
    #     vrep.simxSetJointTargetVelocity(self.client_id, self.wh_hd[i + 3],
    #                                     d, vrep.simx_opmode_oneshot)


    # def init_omni_robot_handle(self, rid):
    #     oneshot_wait = vrep.simx_opmode_oneshot_wait
    #     num = 4 * rid
    #     err = vrep.simx_return_remote_error_flag
    #     while (err != vrep.simx_return_ok) and (not rospy.is_shutdown()):
    #         rname = 'robot#'+str(num)
    #         err, self.rb_hd[rid] = vrep.simxGetObjectHandle(self.client_id,
    #                                                         rname,
    #                                                         oneshot_wait)

    #     rospy.loginfo('vrepsim: init_handle '+rname+'  return: '+str(err))
    #     if err != vrep.simx_return_ok:
    #         return False
    #     for w in range(4):
    #         wnum = num + w - 1
    #         wn = "OmniWheel_regularRotation"
    #         if wnum >= 0:
    #             wn += ('#%d' % wnum)
    #         err = vrep.simx_return_remote_error_flag
    #         while (err != vrep.simx_return_ok) and (not rospy.is_shutdown()):
    #             err, self.wh_hd[num + w] = vrep.simxGetObjectHandle(
    #                                                             self.client_id,
    #                                                             wn,
    #                                                             oneshot_wait)

    #         rospy.loginfo('vrepsim: init_handle '+wn+'  return: '+str(err))
    #         if err != vrep.simx_return_ok:
    #             return False
    #     return True
    