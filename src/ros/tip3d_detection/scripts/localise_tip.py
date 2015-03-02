#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def got_left_image(image_message):
    rospy.loginfo('got left image with width=%s and height=%s', image_message.width, image_message.height)
    
def got_right_image(image_message):
    rospy.loginfo('got right image with width=%s and height=%s', image_message.width, image_message.height)

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
