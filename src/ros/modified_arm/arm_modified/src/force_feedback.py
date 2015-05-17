#!/usr/bin/env python

import roslib
roslib.load_manifest('smart_arm_controller')

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

from numpy import matrix, array
import numpy as np
import sys
import sensor_msgs.msg
from copy import copy, deepcopy
import math

from tip2d_detection_msgs.msg import Tip, Tips

epsilon = 0.05

class ForceFeedbackState:
    # The possible states we can be in
    NORMAL = 0
    FORCEFB = 1

    def __init__(self):
        # P_J - the world position we are commanded to move to
        self.desired_pos = np.ones(3) * np.nan

        # P_C - the current measured position of the arm by camera
        self.camera_pos = np.ones(3) * np.nan
        
        # Store the desired position in this variable
        self.stored_desired_pos = np.ones(3) * np.nan

        # A list containing the last three published positions (/pos_for_IK)
        self.prev_pos = []

        # What state we're in
        self.state = ForceFeedbackState.NORMAL
        
        # A list of publishers to publish state to
        self.pubs = None
        
    def tick(self):
        """Called every so often to update our state."""
        
        if self.state == ForceFeedbackState.NORMAL:
            # send desired position to IK directly
            # Actually publish if we have some publishers
            rospy.logerr('Im in NORMAL state')
            if self.pubs is not None:
                p0 = self.desired_pos
                # Record the current sent pos as element 0 of self.prev_pos
                # shuffling the other pos to the right. Then, truncate the
                # list to be a maximum of 2 pos long.
                self.prev_pos = [p0] + self.prev_pos
                self.prev_pos = self.prev_pos[:2]
            
                #Publish the desired position to /pos_for_IK
                p0_list = p0.tolist()
                msg = Float64MultiArray()
                msg.data = deepcopy(p0_list)
                self.pubs.publish(msg)
            
        elif self.state == ForceFeedbackState.FORCEFB:
            rospy.logerr('Im in FORCEFB state')
            # Calculate correction vector
            
            # Need to check whether desired_pos is nan. Can use one only because the function only allows scalar
            if math.isnan(self.desired_pos[0]) == False:
                self.stored_desired_pos = self.desired_pos
                
            correction = self.stored_desired_pos - self.camera_pos
            correction_mag = np.sqrt(np.sum(correction * correction))
            
            #separate out two terms for testing and simplicity (and also because error in camera measurements in 2 dimensions) 
            correction_x_mag = np.sqrt(correction[0]*correction[0])
            correction_z_mag = np.sqrt(correction[2]*correction[2])
            rospy.logerr('correction_z_mag = ')
            rospy.logerr(correction_z_mag) 
    
            #if the correction_vector bigger and smaller than some threshold, then do something
            #if math.isnan(correction_mag)==True or correction_mag <= epsilon or correction_mag >= 0.05:
            #need correction_z_mag in case we are using force feedback on the wrong pose
            if math.isnan(correction_x_mag)==False and math.isnan(correction_z_mag)==False and correction_z_mag <= 0.15:
                if correction_x_mag >= 0.04 or correction_z_mag >= 0.03:
                    rospy.logerr('FORCE FEEDBACK with correction mag: ' + str(correction_mag))
                
                    # Calculate new commanded pose
                    p0, p1 = self.prev_pos[:2]
                    correction_round = np.zeros(3)
                    # rounding is necessary, otherwise the positions are too specific for IK to work
                    correction_round[0] = round(correction[0],2)
                    correction_round[2] = round(correction[2],2)
                    p0_new = p0 + correction_round
                
                    self.prev_pos = [p0_new] + self.prev_pos
                    self.prev_pos = self.prev_pos[:2]
                    
                    #Publish the new p0 to /pos_for_IK
                    p0_list = p0_new.tolist()
                    msg = Float64MultiArray()
                    msg.data = deepcopy(p0_list)
                    self.pubs.publish(msg)     
                    
                else:
                    rospy.logerr('TRIED FORCE FB BUT NOT REQUIRED')
                    p0 = self.desired_pos
                    # Record the current sent pos as element 0 of self.prev_pos
                    # shuffling the other pos to the right. Then, truncate the
                    # list to be a maximum of 2 pos long.
                    self.prev_pos = [p0] + self.prev_pos
                    self.prev_pos = self.prev_pos[:2]
                
                    #Publish the desired position to /pos_for_IK
                    p0_list = p0.tolist()
                    msg = Float64MultiArray()
                    msg.data = deepcopy(p0_list)
                    self.pubs.publish(msg)
            else:
                rospy.logerr('MATH IS NAN or wrong pose, equivalent to NORMAL state')
                p0 = self.desired_pos
                # Record the current sent pos as element 0 of self.prev_pos
                # shuffling the other pos to the right. Then, truncate the
                # list to be a maximum of 2 pos long.
                self.prev_pos = [p0] + self.prev_pos
                self.prev_pos = self.prev_pos[:2]
            
                #Publish the desired position to /pos_for_IK
                p0_list = p0.tolist()
                msg = Float64MultiArray()
                msg.data = deepcopy(p0_list)
                self.pubs.publish(msg)
                
            self.desired_pos = np.ones(3) * np.nan
            self.state = ForceFeedbackState.NORMAL
        
    def desired_pos_updated(self):
        # We transition to the force feedback state when we have pos and at least 1 previous position
        if len(self.prev_pos) >= 1:
            self.state = ForceFeedbackState.FORCEFB
    
    def desired_pos_normal(self):
        #Transition to normal state
        self.state = ForceFeedbackState.NORMAL

# A global variable holding the controller state
f_state = ForceFeedbackState()

def current_desired_pos(event):
    """Called when a new desired_pos is received."""
    #get the current desired position from /desired_pos (formerly /pos_for_IK)
    #f_state.desired_pos = deepcopy(event.data)
    #switch to force feedback state when command is received from /desired_pos
    f_state.desired_pos[0] = event.data[0]
    f_state.desired_pos[1] = event.data[1]
    f_state.desired_pos[2] = event.data[2]

def current_camera_pos(event):
    #get the x,z coord out of /localise_tip/tip
    f_state.camera_pos[0] = event.x
    f_state.camera_pos[1] = 0 #y is 0 because robot is constrained to move in a plane
    f_state.camera_pos[2] = event.z
    
if __name__ == '__main__':
    pubs = rospy.Publisher('pos_for_IK', Float64MultiArray, queue_size=5)
    rospy.init_node('force_feedback', anonymous=True)
    
    rospy.Subscriber('desired_pos', Float64MultiArray, current_desired_pos)
    rospy.Subscriber('localise_tip/tip', Tip, current_camera_pos)
        
    f_state.pubs = pubs
 
    #every 25 seconds (0.04) & (0.02) to make sure the robot has the correct camera derived position and use 50 secs for big loop
    r = rospy.Rate(0.08) 
    while not rospy.is_shutdown():
        f_state.desired_pos_updated()
        f_state.tick()
        r.sleep()
        

