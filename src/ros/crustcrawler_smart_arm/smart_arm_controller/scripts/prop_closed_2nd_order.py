#!/usr/bin/env python
#remove rospy.spin() and subscriber for this code to work.. how to get subscriber to work?

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
current_pose = matrix(((-0.9, 1.972222, -1.972222, 0.0, 0.0),
                       (-0.9, 1.972222, -1.972222, 0.0, 0.0),
                       (-0.9, 1.972222, -1.972222, 0.0, 0.0)))
#goal pose and step size
#goal pose -- I am not sure why I need 2 lines .. Why delay? The first row is the goal position you want to achieve
#The second row is just the initial position -- listen in practice
joint_commands_goal = matrix(((-0.9, 1.1, -1.25, 0.0, 0.0),
                              (0.0, 0.0, 0.0, 0.0, 0.0)))

#Poles should be on the real axis to avoid overshoot and oscillations
r = 0.95
alpha = r*r
k = 1+r*r-2*r*0.9986

def callback(data):
    a = data.position
    
    current_pose[0,0] = a[3]
    current_pose[0,1] = a[1]
    current_pose[0,2] = a[4]
    current_pose[0,3] = a[2]
    current_pose[0,4] = a[0]
    
    current_pose[1,0] = a[3]
    current_pose[1,1] = a[1]
    current_pose[1,2] = a[4]
    current_pose[1,3] = a[2]
    current_pose[1,4] = a[0]
    
    current_pose[2,0] = a[3]
    current_pose[2,1] = a[1]
    current_pose[2,2] = a[4]
    current_pose[2,3] = a[2]
    current_pose[2,4] = a[0]
    
def listen_callback(event):
    #pose = current_pose
    rospy.logerr(current_pose[0])
   
if __name__ == '__main__':
    pubs = [rospy.Publisher(name + '/command', Float64) for name in joint_names]
    rospy.init_node('make_goal_pose', anonymous=True)
        
    rospy.sleep(5)
    
    #joint_commands_goal must come after current_pose
    pose = current_pose
    joint_commands_goal[1] = pose[0]
    
    #rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, callback)
    #rospy.Timer(rospy.Duration(0.01), listen_callback)
    
    for i in range(len(pubs)):
                pubs[i].publish(pose[0,i])
                
    rospy.sleep(5)
    
    #Control the robot to the goal position
    r = rospy.Rate(30)
    
    f = 0
    
    while not rospy.is_shutdown():

        #Break or stop the command when the current pose is sufficiently close to the desired pose
        delta = joint_commands_goal[0] - pose[0]
        delta_mag = np.linalg.norm(delta)

        epsilon = 0.01

        if f>400 and delta_mag <= epsilon or delta_mag >= 100:
            break
            
        #Second order controller
        else:
            pose[2] = pose[1]
            pose[1] = pose[0]
            joint_commands_goal[1] = joint_commands_goal[0]
        
            pose[0] = (1+alpha-k)*pose[1] - alpha*pose[2] + k*joint_commands_goal[1]
            
            f += 1
            
            for i in range(len(pubs)):
                pubs[i].publish(pose[0,i])
            
            r.sleep()
    
    #rospy.spin()   
    rospy.spin()
    sys.exit(0)
