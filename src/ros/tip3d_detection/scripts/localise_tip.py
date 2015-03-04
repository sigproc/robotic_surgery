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
from sensor_msgs.msg import Image
from tip3d_detection_msgs.msg import Tip
from tip3d_detection import image_to_array, TipDetector, world_coordinates

import numpy as np

STATE={"left":None,"right":None,"tip_publisher":None,"last_image_header":None}

def handle_images():
    tip_msg = Tip()

    # Set "header" for tip msg
    tip_msg.header = STATE["last_image_header"]
    
    # TODO: process STATE["left"] and STATE["right"] and get actual tip location
    # Detect tips in the image
    detect_tip = TipDetector
    tips2d = detect_tip(STATE["left"])

    # Convert to robot's world coordinates
    convert_world = world_coordinates
    tips3d = convert_world(tips2d[0],tips2d[1],STATE["left"],STATE["right"])
    
    # Set positions to real values
    tip_msg.x = tips3d[0] #np.random.random()
    tip_msg.y = tips3d[1] #np.random.random()
    tip_msg.z = tips3d[2] #np.random.random()

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
