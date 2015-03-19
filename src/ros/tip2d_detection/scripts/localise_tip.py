#!/usr/bin/env python
# The MIT License (MIT)
#
# Copyright (c) 2015 Duong Le <tdl28@cam.ac.uk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
import rospy
import roslib
import sys
import math
import cv2
from sensor_msgs.msg import Image
from tip2d_detection_msgs.msg import Tip, Tips
from tip2d_detection import image_to_array, TipDetector, world_coordinates

import numpy as np

STATE={"image":None,"tip_publisher":None,"last_image_header":None}

def handle_images():
    tip_msg = Tip()

    # Set "header" for tip msg
    tip_msg.header = STATE["last_image_header"]
    
    # TODO: process STATE["image"] and get actual tip location
    # Detect tips in the image
    detect_tip = TipDetector
    image = STATE["image"]
    tips = detect_tip(image)
    
    #tips = (320,240)
    
    # print coordinates
    rospy.logerr('Coordinates of tip: %s',
            tips)

    if tips[0] == 0 and tips[1] == 0:
        rospy.logerr('Saving troublesome image')
        cv2.imwrite('/home/ros/workspace/src/robotic_surgery/tip2d_detection/image.png',image)
        #np.save('home/ros/workspace/src/robotic_surgery/tip2d_detection/image.png', image)
    else:   
        # Convert to robot's world coordinates
        convert_world = world_coordinates
        tips2d = convert_world(tips[0],tips[1])
        
        # Set positions to real values
        tip_msg.x = tips2d[0] #np.random.random()
        tip_msg.z = tips2d[1] #np.random.random()
        
        # Publish the tip message to the other nodes which are interested
        STATE["tip_publisher"].publish(tip_msg)

def got_image(image_message):
    STATE["last_image_header"]=image_message.header
    STATE["image"]=image_to_array(image_message)
    if STATE["image"] is not None:
        handle_images()
    
def main():
    # Register the node with ROS
    rospy.init_node('localise_tip', anonymous=True)
    
    # Subscribe to the image
    rospy.Subscriber("image", Image, got_image)
    
    # Create a publisher for tip locations
    STATE["tip_publisher"] = rospy.Publisher(rospy.get_name() + '/tip', Tip, queue_size=1)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__=='__main__':
    main()
