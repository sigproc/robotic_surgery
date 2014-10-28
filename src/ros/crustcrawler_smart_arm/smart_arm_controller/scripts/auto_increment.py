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
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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

import numpy as np

joint_names = ('shoulder_pan_controller',
               'shoulder_pitch_controller',
               'elbow_flex_controller',
               'wrist_roll_controller',
               'claw_controller')
               
joint_commands = [
        (3.14, 1.9722, -0.5, 0.0, 0.0),
        (3.14, 1.9722, -1.27, 0.0, 0.0),
        (3.14, 1.9722, 0.2, 0.0, 0.0),
        (3.14, 1.9722, -2.2, 0.0, 0.0),
        (3.14, 1.9722, -2.2, 0.0, 0.0),
        (3.14, 1.9722, -2.2, 0.0, 0.0)
]

#creating a matrix c that contains one column in joint_commands 
#c_diff that calculates the difference between one value and the next
length_of_commands = len(joint_commands)
c = np.zeros((length_of_commands,1))
c_diff = np.zeros((length_of_commands,1))
for i in range (0,length_of_commands):
    c[i][0]=joint_commands[i][2]
    c_diff[i][0]=c[i][0]-c[i-1][0]
    i += 1

#increment step size
g = 0.01 

#determine whether + or - by comparing the next position with current position
if (c_diff[1][0]<=c_diff[0][0]):
    n = -g
else: n = g
       
#create a new matrix joint_commands_smooth and iterate based on the step size for one joint     
b = int ((c[1][0]-c[0][0])/g)
k = 0
joint_commands_smooth = joint_commands
for k in range (0,abs(b)):
    joint_commands_smooth = np.insert(joint_commands_smooth,k+1,np.array((3.14,1.9722,joint_commands_smooth[k][2]+n,0,0)),0)
    k += 1
    
###PART 2 ###    
#update c_diff and run the next positions   
iteration = 2  
while iteration in range (2,length_of_commands):
    u = len(joint_commands_smooth)
    c = np.zeros((u,1))
    c_diff = np.zeros((u,1))
    for i in range (0,u):
        c[i][0]=joint_commands_smooth[i][2]
        c_diff[i][0]=c[i][0]-c[i-1][0]
        i += 1    

    #iterate for next position
    if (c_diff[k+1][0]<=c_diff[k][0]):
        n = -g
    else: n = g
       
    #create a new matrix joint_commands_smooth and iterate based on the step size for one joint
    k += 1
    b = int ((c[k+1][0]-c[k][0])/g)
    for k in range (k,abs(b)+k):
        joint_commands_smooth = np.insert(joint_commands_smooth,k+1,np.array((3.14,1.9722,joint_commands_smooth[k][2]+n,0,0)),0)
        k += 1
        
    iteration += 1

if __name__ == '__main__':
    pubs = [rospy.Publisher(name + '/command', Float64) for name in joint_names]
    rospy.init_node('make_cobra_pose', anonymous=True)
    
    r = rospy.Rate(30)
    idx = 0
    while not rospy.is_shutdown():
        for i in range(len(pubs)):
            pubs[i].publish(joint_commands_smooth[idx][i])
        idx += 1
        if idx >= len(joint_commands_smooth):
            idx = 0

        r.sleep()
