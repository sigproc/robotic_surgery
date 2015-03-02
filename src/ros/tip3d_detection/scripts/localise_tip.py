#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

STATE={"left":None,"right":None}

def handle_images():
    rospy.loginfo("Left: %s, Right: %s",STATE["left"].width, STATE["right"].width)

def got_left_image(image_message):
    STATE["left"]=image_message
    if STATE["left"] is not None and STATE["right"] is not None:
        handle_images()  
    
def got_right_image(image_message):
    STATE["right"]=image_message
    if STATE["left"] is not None and STATE["right"] is not None:
        handle_images()
    
def main():
    # Register the node with ROS
    rospy.init_node('localise_tip', anonymous=True)
    
    # Subscribe to left and right images
    rospy.Subscriber("left_image", Image, got_left_image)
    rospy.Subscriber("right_image", Image, got_right_image)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__=='__main__':
    main()
