#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Arizona Robotics Research Group,
# University of Arizona. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of University of Arizona nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib
roslib.load_manifest('smart_arm_controller')

import rospy

from std_msgs.msg import Float64

from numpy import matrix
import numpy as np

joint_names = ('shoulder_pan_controller',
'shoulder_pitch_controller',
'elbow_flex_controller',
'wrist_roll_controller',
'left_finger_controller')

#Initialise, measure in practice
pose = matrix((-0.9, 1.972222, -1.972222, 0.0, 0.0))
#goal pose and step size
joint_commands_goal = matrix((0.5, 1.1, -0.51, -1.97222, 1.0))
kappa = 0.01

if __name__ == '__main__':
    pubs = [rospy.Publisher(name + '/command', Float64) for name in joint_names]
    rospy.init_node('make_goal_pose', anonymous=True)
    
    #publish the initial state to the robot first and sleep for 8 secs
    for i in range(len(pubs)):
        pubs[i].publish(pose[0,i])
        
    rospy.sleep(8)
    
    #Control the robot to the goal position
    r = rospy.Rate(10)
    idx = 0

    while not rospy.is_shutdown():
        delta = joint_commands_goal - pose
        delta_mag = np.linalg.norm(delta)

        epsilon = 1

        if delta_mag <= epsilon:
            break
    
        else:
            joint_commands_smooth = pose + kappa*delta
            
            for i in range(len(pubs)):
                pubs[i].publish(joint_commands_smooth[0,i])
                
            pose = joint_commands_smooth

            r.sleep()
