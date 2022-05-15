#!/usr/bin/env python3
import rospy
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ParamValue, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from tf.transformations import *

from math import *
import numpy as np
from numpy.linalg import norm
import time


class Drone:
    def __init__(self):
        self.pose = None
        self.yaw = np.pi/2
        self.sp = None
        self.hz = 25
        self.rate = rospy.Rate(self.hz)

        self.current_state = State()
        self.extended_state = ExtendedState()

        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # ROS services
        service_timeout = 30
        rospy.loginfo("Waiting for ROS services")
        try:
            rospy.wait_for_service('/mavros/param/set', service_timeout)
            rospy.wait_for_service('/mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('/mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_param_client = rospy.ServiceProxy('mavros/param/set', ParamSet)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
        rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_callback)

    def __set_param(self, param_id, param_value: ParamValue, timeout: float) -> bool:
        """param: PX4 param string, ParamValue, timeout(int): seconds"""
        if param_value.integer != 0:
            value = param_value.integer
        else:
            value = param_value.real
        rospy.loginfo("Setting PX4 parameter: {0} with value {1}".
        format(param_id, value))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            try:
                res = self.set_param_client(param_id, param_value)
                if res.success:
                    rospy.loginfo("Param {0} set to {1} | seconds: {2} of {3}".
                    format(param_id, value, i / loop_freq, timeout))
                return True
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        rospy.logerr(
            "Failed to set param | param_id: {0}, param_value: {1} | timeout(seconds): {2}".
            format(param_id, value, timeout))
        return False

    def __set_mode(self, mode, timeout) -> bool:
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("Setting FCU mode: {0}".format(mode))
        old_mode = self.current_state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if self.current_state.mode == mode:
                rospy.loginfo("Set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                return True
            else:
                try:
                    res = self.set_mode_client(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("Failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        rospy.logerr(
            "Failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
            format(mode, old_mode, timeout))
        return False

    def state_callback(self, state):
        if self.current_state.armed != state.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.current_state.armed, state.armed))

        if self.current_state.connected != state.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.current_state.connected, state.connected))

        if self.current_state.mode != state.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.current_state.mode, state.mode))

        if self.current_state.system_status != state.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.current_state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][state.system_status].name))
        self.current_state = state

    def extended_state_callback(self, data):
        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("Landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

    def drone_pose_callback(self, pose_msg):
        self.pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])

    def arm(self):
        for i in range(self.hz):
            self.publish_setpoint([0,0,-1])
            self.rate.sleep()
    
        # wait for FCU connection
        while not self.current_state.connected:
            rospy.loginfo('Waiting for FCU connection...')
            self.rate.sleep()
        
        ## JMAVSIM ONLY
        # exempting failsafe from lost RC to allow offboard
        # rcl_except = ParamValue(1<<2, 0.0)
        # self.__set_param("COM_RCL_EXCEPT", rcl_except, 5)

        prev_request = rospy.get_time()
        prev_state = self.current_state
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.current_state.mode != "OFFBOARD" and (now - prev_request > 2.):
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                prev_request = now 
            else:
                if not self.current_state.armed and (now - prev_request > 2.):
                   self.arming_client(True)
                   prev_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != self.current_state.armed:
                rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)

            if prev_state.mode != self.current_state.mode: 
                rospy.loginfo("Current mode: %s" % self.current_state.mode)
            prev_state = self.current_state

            if self.current_state.armed:
                break
            # Update timestamp and publish sp 
            self.publish_setpoint([0,0,-1])
            self.rate.sleep()

    @staticmethod
    def get_setpoint(x, y, z, yaw=np.pi/2):
        set_pose = PoseStamped()
        set_pose.pose.position.x = x
        set_pose.pose.position.y = y
        set_pose.pose.position.z = z
        q = quaternion_from_euler(0, 0, yaw)
        set_pose.pose.orientation.x = q[0]
        set_pose.pose.orientation.y = q[1]
        set_pose.pose.orientation.z = q[2]
        set_pose.pose.orientation.w = q[3]
        return set_pose
    def publish_setpoint(self, sp, yaw=np.pi/2):
        #print(yaw)
        setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw)
        setpoint.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(setpoint)
    def publish_pose(self, pose: PoseStamped):
        self.yaw = euler_from_quaternion([  pose.pose.orientation.x,
                                            pose.pose.orientation.y,
                                            pose.pose.orientation.z,
                                            pose.pose.orientation.w])[2]
        pose.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(pose)

    def takeoff(self, height):
        rospy.loginfo("Takeoff...")
        # self.__set_mode("AUTO.TAKEOFF", 5)
        # takeoff_state_confirmed = False
        # while not takeoff_state_confirmed:
        #     if self.extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
        #         takeoff_state_confirmed = True
        #     try:
        #         self.rate.sleep()
        #     except rospy.ROSException as e:
        #         self.fail(e)
        self.sp = self.pose
        while not rospy.is_shutdown() and self.pose[2] < height:
            if rospy.is_shutdown():
                self.land()
            self.sp[2] += 0.02
            self.publish_setpoint(self.sp, self.yaw)
            self.rate.sleep()

    def hover(self, t_hold):
        rospy.loginfo('Position holding...')
        t0 = time.time()
        self.sp = self.pose
        while not rospy.is_shutdown():
            t = time.time()
            if t - t0 > t_hold and t_hold > 0: break
            # Update timestamp and publish sp 
            self.publish_setpoint(self.sp, self.yaw)
            self.rate.sleep()

    def land(self):
        rospy.loginfo("Landing...")
        self.__set_mode("AUTO.LAND", 5)
        landed_state_confirmed = False
        while not landed_state_confirmed:
            if self.extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                landed_state_confirmed = True
            
            try:
                self.rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
        self.stop()

    def stop(self):
        while self.current_state.armed or self.current_state.mode == "OFFBOARD":
            if self.current_state.armed:
                self.arming_client(False)
            if self.current_state.mode == "OFFBOARD":
                self.set_mode_client(base_mode=0, custom_mode="MANUAL")
            self.rate.sleep()

    @staticmethod
    def transform(pose):
        # transformation: x - froward, y - left, z - up (ENU - MoCap frame)
        pose_new = np.zeros(3)
        pose_new[0] = - pose[1]
        pose_new[1] = pose[0]
        pose_new[2] = pose[2]
        return pose_new

    def goTo(self, wp, mode='global', tol=0.05):
        wp = self.transform(wp)
        if mode=='global':
            goal = wp
        elif mode=='relative':
            goal = self.pose + wp
        rospy.loginfo("Going to a waypoint...")
        self.sp = self.pose
        while not rospy.is_shutdown() and norm(goal - self.pose) > tol:
            if rospy.is_shutdown():
                self.land()
            n = (goal - self.sp) / norm(goal - self.sp)
            self.sp += 0.03 * n
            #print(self.pose, goal)
            self.publish_setpoint(self.sp, self.yaw)
            self.rate.sleep()
