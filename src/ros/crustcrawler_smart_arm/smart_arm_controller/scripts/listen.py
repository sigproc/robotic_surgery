#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('smart_arm_controller')

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from numpy import matrix
import numpy as np
import sys
import sensor_msgs.msg

joint_names = ('shoulder_pan_controller',
'shoulder_pitch_controller',
'elbow_flex_controller',
'wrist_roll_controller',
'claw_controller')

LOG_FILE_NAME='/home/ros/armlog.txt'

log_file = open(LOG_FILE_NAME, 'w')

def log_command(event, name):
    """Called when a new command is sent to the arm."""
    angle = event.data
    when = rospy.Time.now().to_sec()
    log_file.write('SEND:{1}:{2}:{0}\n'.format(angle, when, name))

def log_state(event):
    when = event.header.stamp.to_sec()
    for name, pos, vel, eff in zip(event.name, event.position, event.velocity, event.effort): 
        log_file.write('JOINT:{0}:{1}:{2}:{3}:{4}\n'.format(when, name, pos, vel, eff))

def main():
    rospy.init_node('listener')
    for jn in joint_names:
        rospy.Subscriber(jn + '/command', Float64, log_command, callback_args=jn)
    rospy.Subscriber('/joint_states', JointState, log_state)
    rospy.spin()

if __name__ == '__main__':
    main()    
    
