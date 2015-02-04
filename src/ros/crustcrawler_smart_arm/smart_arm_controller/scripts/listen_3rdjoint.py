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

LOG_FILE_NAME_1='/home/ros/elbowflexlog_position.txt'
log_file_1 = open(LOG_FILE_NAME_1, 'w')

LOG_FILE_NAME_2='/home/ros/elbowflexlog_velocity.txt'
log_file_2 = open(LOG_FILE_NAME_2, 'w')

LOG_FILE_NAME_3='/home/ros/elbowflex_command.txt'
log_file_3 = open(LOG_FILE_NAME_3, 'w')

def log_command(event, name):
    """Called when a new command is sent to the arm."""
    angle = event.data
    when = rospy.Time.now().to_sec()
    #log_file_3.write('{0}\n'.format(angle))
    if (name == 'elbow_flex_controller'):
        log_file_3.write('{0}:{1}\n'.format(when, angle))

def log_state(event):
    when = event.header.stamp.to_sec()
    #not sure this will work
    for name, pos, vel, eff in zip(event.name, event.position, event.velocity, event.effort): 
        if (name == 'elbow_flex_joint'):
            #log_file_1.write('{0}\n'.format(event.position))
            #log_file_2.write('{0}\n'.format(event.velocity))
            log_file_1.write('{0}:{1}\n'.format(when, pos))
            log_file_2.write('{0}:{1}\n'.format(when, vel))

def main():
    rospy.init_node('listener')
    for jn in joint_names:
        rospy.Subscriber(jn + '/command', Float64, log_command, callback_args=jn)
    rospy.Subscriber('/joint_states', JointState, log_state)
    rospy.spin()

if __name__ == '__main__':
    main()    
    
