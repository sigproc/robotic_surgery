#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from tip3d_detection_msgs.msg import Tip
from tip3d_detection import image_to_array

import numpy as np

STATE={"left":None,"right":None,"tip_publisher":None,"last_image_header":None}

def handle_images():
    tip_msg = Tip()

    # Set "header" for tip msg
    tip_msg.header = STATE["last_image_header"]
    
    # TODO: process STATE["left"] and STATE["right"] and get actual tip location
    
    # Set position to fake values
    tip_msg.x = STATE["left"].shape[0]
    tip_msg.y = STATE["right"].shape[0]
    tip_msg.z = np.random.random()

    # Publish the tip message to the other nodes which are interested
    STATE["tip_publisher"].publish(tip_msg)

def got_image(side, image_message):
    STATE["last_image_header"]=image_message.header
    STATE[side]=image_to_array(image_message)
    if STATE["left"] is not None and STATE["right"] is not None:
        handle_images()
        
def got_left_image(image_message):
    got_image('left', image_message)
    
def got_right_image(image_message):
    got_image('right', image_message)
    
def main():
    # Register the node with ROS
    rospy.init_node('localise_tip', anonymous=True)
    
    # Subscribe to left and right images
    rospy.Subscriber("left_image", Image, got_left_image)
    rospy.Subscriber("right_image", Image, got_right_image)
    
    # Create a publisher for tip locations
    STATE["tip_publisher"] = rospy.Publisher(rospy.get_name() + '/tip', Tip, queue_size=1)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__=='__main__':
    main()
