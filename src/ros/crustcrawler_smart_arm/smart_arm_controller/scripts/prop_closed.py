#!/usr/bin/env python

import roslib
roslib.load_manifest('smart_arm_controller')

import rospy

from std_msgs.msg import Float64

from numpy import matrix
import numpy as np
import sys
import sensor_msgs.msg

joint_names = ('shoulder_pan_controller',
'shoulder_pitch_controller',
'elbow_flex_controller',
'wrist_roll_controller',
'claw_controller')

#    Use rospy.logerr(data.current_pos) to log data

#Initialise, measure in practice
current_pose = matrix((-0.9, 1.972222, -1.972222, 0.0, 0.0))
#goal pose and step size
joint_commands_goal = matrix((0.5, 1.1, -0.91, -1.97222, 1.0))
kappa = 0.01

def callback(data):
    a = data.position
    current_pose[0,0] = a[3]
    current_pose[0,1] = a[1]
    current_pose[0,2] = a[4]
    current_pose[0,3] = a[2]
    current_pose[0,4] = a[0]
    
def listen_callback(event):
    pose = current_pose
   
if __name__ == '__main__':
    pubs = [rospy.Publisher(name + '/command', Float64) for name in joint_names]
    rospy.init_node('make_goal_pose', anonymous=True)
    
    rospy.sleep(5)
    
    #Control the robot to the goal position
    r = rospy.Rate(10)
    idx = 0
    
    rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, callback)
    
    pose=current_pose
    
    #rospy.Timer(rospy.Duration(3), listen_callback)
    
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
            
            rospy.logerr(pose)

            r.sleep()

    rospy.spin()        
    rospy.spin()
    sys.exit(0)
