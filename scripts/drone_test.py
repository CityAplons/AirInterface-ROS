#!/usr/bin/env python3

import rospy
import numpy as np
from drone import Drone


rospy.init_node('drone_control', anonymous=True)
drone = Drone()
drone.arm()
drone.takeoff(1.0)
drone.hover(1.0)
drone.goTo([0.5,0.,0.], mode='relative')
drone.hover(1.0)
drone.goTo([-0.5,0.,0.], mode='relative')
drone.hover(1.0)
drone.goTo([0.,0.5,0.], mode='relative')
drone.hover(1.0)
drone.goTo([0.,-0.5,0.], mode='relative')
drone.hover(1.0)
drone.yaw = 0
drone.hover(5.0)
drone.yaw = -np.pi/2
drone.hover(5.0)
drone.yaw = np.pi
drone.hover(3.0)
drone.land()