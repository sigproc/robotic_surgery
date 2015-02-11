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

desired_position = [0.1285793, 0, 0.469884]

if __name__ == '__main__':
    pubs = rospy.Publisher('pos_for_IK', Float64MultiArray, queue_size=10)
    rospy.init_node('position_for_IK', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
    
        msg = Float64MultiArray()
        msg.data = deepcopy(desired_position)
        pubs.publish(msg)

        rate.sleep()

