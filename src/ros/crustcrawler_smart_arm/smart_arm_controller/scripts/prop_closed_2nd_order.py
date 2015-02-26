#!/usr/bin/env python

import roslib
roslib.load_manifest('smart_arm_controller')
import rospy
from std_msgs.msg import Float64
from numpy import matrix, array
import numpy as np
import sys
import sensor_msgs.msg
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointState

joint_names = ('shoulder_pan_controller',
'shoulder_pitch_controller',
'elbow_flex_controller',
'wrist_roll_controller',
'claw_controller')

#Poles should be on the real axis to avoid overshoot and oscillations
r = 0.95
alpha = r*r
k = 1+r*r-2*r*0.9986

class PropControllerState:
    # The possible states we can be in
    LISTENING = 0
    CONTROLLING = 1

    def __init__(self):
        # P_J - the pose we are commanded to move to
        self.commanded_pose = np.ones(5) * np.nan

        # P_C - the current measured pose of the arm
        self.measured_pose = np.ones(5) * np.nan

        # A list containing the last three measured states.
        self.prev_poses = []

        # What state we're in
        self.state = PropControllerState.LISTENING
        
        # A list of publishers to publish state to
        self.pubs = None
        
    def tick(self):
        """Called every so often to update our state."""
        
        if self.state == PropControllerState.LISTENING:
            # If we have all of one pose...
            if np.all(np.isfinite(self.measured_pose)):
                # Record the current pose as element 0 of self.prev_poses
                # shuffling the other poses to the right. Then, truncate the
                # list to be a maximum of 3 poses long.
                self.prev_poses = [np.copy(self.measured_pose)] + self.prev_poses
                self.prev_poses = self.prev_poses[:3]
        elif self.state == PropControllerState.CONTROLLING:
            # Calculate new commanded pose
            p0, p1, p2 = self.prev_poses[:3]
            p0 = (1+alpha-k)*p0 - alpha*p1 + k*self.commanded_pose
            #p0 = 0.8*p1 + 0.2*self.commanded_pose
            
            # Shuffle prev poses array
            self.prev_poses = [p0] + self.prev_poses
            self.prev_poses = self.prev_poses[:3]

            delta = self.commanded_pose - p0
            delta_mag = np.sqrt(np.sum(delta * delta))
            epsilon = 0.02

            #rospy.logerr('p0: '+ str(p0))
            #rospy.logerr('p1: '+ str(p1))
            #rospy.logerr('p2: '+ str(p2))
            rospy.logerr('Cmded pose: ' + str(self.commanded_pose))
            #rospy.logerr('delta: ' +str(delta))
            #rospy.logerr('Controlling with prev poses: ' + str(self.prev_poses))

            # If we're sufficiently close, or have gone mad, transition to listening
            # state and reset the commanded pose.
            
            #The IK works for point (0.1, 0, 0) but it is not publishing
            if delta_mag <= epsilon or delta_mag >= 6:
                #rospy.logerr('Finished with delta mag: ' + str(delta_mag))
                self.commanded_pose = np.ones(5) * np.nan
                self.state = PropControllerState.LISTENING
            elif self.pubs is not None:
                # Actually publish *IF* we have some publishers
                for i in range(len(self.pubs)):
                    self.pubs[i].publish(p0[i])
        
    def commanded_pose_updated(self):
        # We transition to the controlling state when we have a full commanded
        # pose *AND* at least three previous poses
        if np.all(np.isfinite(self.commanded_pose)) and len(self.prev_poses) >= 3:
            self.state = PropControllerState.CONTROLLING

# A global variable holding the controller state
c_state = PropControllerState()

def get_shoulder_pan(event):
    """Called when a new command is sent to the arm."""
    c_state.commanded_pose[0] = event.data
    c_state.commanded_pose_updated()
    
def get_shoulder_pitch(event):
    """Called when a new command is sent to the arm."""
    c_state.commanded_pose[1] = event.data
    c_state.commanded_pose_updated()
   
def get_elbow_flex(event):
    """Called when a new command is sent to the arm."""
    c_state.commanded_pose[2] = event.data
    c_state.commanded_pose_updated()

def get_wrist_roll(event):
    """Called when a new command is sent to the arm."""
    c_state.commanded_pose[3] = event.data
    c_state.commanded_pose_updated()

def get_claw(event):
    """Called when a new command is sent to the arm."""
    #Claw should always be closed
    c_state.commanded_pose[4] = 1.9722
    c_state.commanded_pose_updated()

def current_shoulder_pan(event):
    c_state.measured_pose[0] = event.current_pos

def current_shoulder_pitch(event):
    c_state.measured_pose[1] = event.current_pos
    
def current_elbow_flex(event):
    c_state.measured_pose[2] = event.current_pos
    
def current_wrist_roll(event):
    c_state.measured_pose[3] = event.current_pos
    
def current_claw(event):
    c_state.measured_pose[4] = event.current_pos

if __name__ == '__main__':
    pubs = [rospy.Publisher(name + '/command', Float64) for name in joint_names]
    rospy.init_node('make_goal_pose', anonymous=True)
    
    rospy.Subscriber('shoulder_pan_controller/state', JointState, current_shoulder_pan)   
    rospy.Subscriber('shoulder_pitch_controller/state', JointState, current_shoulder_pitch)
    rospy.Subscriber('elbow_flex_controller/state', JointState, current_elbow_flex)
    rospy.Subscriber('wrist_roll_controller/state', JointState, current_wrist_roll)    
    rospy.Subscriber('claw_controller/state', JointState, current_claw)   
    
    rospy.Subscriber('shoulder_pan_controller/intermediate_command', Float64, get_shoulder_pan)   
    rospy.Subscriber('shoulder_pitch_controller/intermediate_command', Float64, get_shoulder_pitch)
    rospy.Subscriber('elbow_flex_controller/intermediate_command', Float64, get_elbow_flex)
    rospy.Subscriber('wrist_roll_controller/intermediate_command', Float64, get_wrist_roll)    
    rospy.Subscriber('claw_controller/intermediate_command', Float64, get_claw)

    c_state.pubs = pubs
 
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        c_state.tick()
        r.sleep()
