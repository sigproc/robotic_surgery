#!/usr/bin/env python

import roslib
roslib.load_manifest('smart_arm_controller')

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

from numpy import matrix
import numpy as np
import sys
import sensor_msgs.msg
from copy import copy, deepcopy

#The alternating box positions for demo
#desired_positions = [
#    [0.225307, 0.0, 0.210856],
#    [0.225307, 0.0, 0.310856],
#    [0.325307, 0.0, 0.310856],
#    [0.325307, 0.0, 0.210856]
#]

desired_positions = [
    [0.30, 0.0, 0.05],
    [0.30, 0.0, 0.15],
    [0.30, 0.0, 0.25],
    [0.32, 0.0, 0.15],
    [0.30, 0.0, 0.15],
    [0.28, 0.0, 0.15]
]

if __name__ == '__main__':
    pubs = rospy.Publisher('pos_for_IK', Float64MultiArray, queue_size=5)
    rospy.init_node('position_for_IK', anonymous=True)
    
    rate = rospy.Rate(0.05) #once every 20 secs
    
    pos_idx = 0
    while not rospy.is_shutdown():
        pos_idx += 1
        if pos_idx >= len(desired_positions):
            pos_idx = 0
    
        msg = Float64MultiArray()
        msg.data = deepcopy(desired_positions[pos_idx])
        pubs.publish(msg)

        rate.sleep()

