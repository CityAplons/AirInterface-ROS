#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from airinterface.srv import UnityGetState, UnityGetStateResponse, UnitySetState, UnitySetStateRequest, UnitySetStateResponse
import std_srvs.srv

import numpy as np
from drone import Drone

from datetime import datetime
from pathlib import Path

class UnityDroneController:
    
    drone_state = UnityGetStateResponse.ONGROUND
    mode_state = UnitySetStateRequest.LAND

    def __init__(self) -> None:
        rospy.init_node('drone_control', anonymous=True)
        
        ## Delay configuration resolver
        self.make_delays = False
        if rospy.has_param('~delays'):
            self.make_delays = rospy.get_param('~delays')
        self.delays_thresh = 500
        self.delays_dir = str(Path.home())
        if self.make_delays:
            rospy.logwarn("Delay measurement is ON")
            if rospy.has_param('~delays_dir'):
                self.delays_dir = rospy.get_param('~delays_dir')
                Path(self.delays_dir).mkdir(parents=True, exist_ok=True)
            if rospy.has_param('~delays_thresh'):
                self.delays_thresh = rospy.get_param('~delays_thresh')
                rospy.logwarn("Delay measurement threshold changed to %d"%self.delays_thresh)
            rospy.logwarn("Delay measurements directory: %s"%Path(self.delays_dir).resolve())

        rospy.Service('unity/get_state', UnityGetState, self.handle_drone_state)
        rospy.Service('arm_active', std_srvs.srv.Trigger, self.handle_arm_state)
        rospy.Service('unity/set_state', UnitySetState, self.handle_drone_mode)
        rospy.Subscriber('unity/set_pose', PoseStamped, self.set_pose_callback, queue_size=1)
        self.drone = Drone()
        self.states = {
            UnitySetStateRequest.TAKEOFF: self.__takeoff,
            UnitySetStateRequest.HOVER: self.__hover,
            UnitySetStateRequest.POSCTL: self.__position_control,
            UnitySetStateRequest.LAND: self.__land
        }
        self.set_point = None # hover set_point
        self.pose = None # movto pose
        self.timestamps = []
        self.delays = np.array([[0., 0.]])
        self.control_provider()

    def handle_arm_state(self, req):
        if self.drone_state == UnityGetStateResponse.INAIR and self.drone.current_state.mode == "OFFBOARD" and \
            (self.mode_state == UnitySetStateRequest.HOVER or self.mode_state == UnitySetStateRequest.POSCTL):
            return std_srvs.srv.TriggerResponse(True, "Drone Ready")
        else:
            return std_srvs.srv.TriggerResponse(False, "Drone Busy")

    def handle_drone_state(self, req):
        return self.drone_state

    def handle_drone_mode(self, req):
        self.mode_state = req.set_state
        return UnitySetStateResponse(True)

    def set_pose_callback(self, data):
        self.timestamps.append((rospy.Time.now() - data.header.stamp).to_sec())
        if self.make_delays and len(self.timestamps) >= self.delays_thresh:
            delay = np.mean(self.timestamps)
            jitter = np.std(np.array(self.timestamps))
            self.delays = np.append(self.delays, [[delay, jitter]], axis=0)
            rospy.logwarn("=== Time delay %f, jitter %f ===", delay, jitter)
            self.timestamps = []
        self.pose = data

    def control_provider(self):
        prev_state = self.mode_state
        while not rospy.is_shutdown():
            if self.mode_state != prev_state:
                rospy.loginfo("[SM] Mode changed to %s", self.mode_state)
                prev_state = self.mode_state
            if self.drone.current_state.mode != "OFFBOARD":
                self.drone_state = UnityGetStateResponse.ONGROUND
            self.states[self.mode_state]()
            self.drone.rate.sleep()

    def __takeoff(self):
        if self.drone_state == UnityGetStateResponse.ONGROUND:
            self.drone.arm()
            self.drone.takeoff(1.5)
            self.drone_state = UnityGetStateResponse.INAIR
            self.mode_state = UnitySetStateRequest.HOVER

    def __hover(self):
        if self.set_point is None: 
            self.set_point = self.drone.pose
        self.drone.publish_setpoint(self.set_point, self.drone.yaw)
    
    def __position_control(self):
        if self.pose is not None:
            self.set_point = None
        else:
            sp = self.drone.pose
            self.pose = self.drone.get_setpoint(sp[0], sp[1], sp[2])
        self.drone.publish_pose(self.pose)

    def __land(self):
        if self.drone_state == UnityGetStateResponse.INAIR:
            self.drone.land()
            self.drone_state = UnityGetStateResponse.ONGROUND
            self.mode_state = UnitySetStateRequest.LAND
            
            if self.make_delays:
                now = datetime.now()
                np.savetxt('%s/delays.%s.txt'%(self.delays_dir, now.strftime("%d-%m-%Y-%H-%M-%S")), self.delays, delimiter=",", fmt="%.6f")
                self.delays = np.array([[0, 0]])

if __name__ == '__main__':
    try:
        UnityDroneController()
    except rospy.ROSInterruptException:
        pass
