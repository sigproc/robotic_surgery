#!/usr/bin/env python
#
# The MIT License (MIT)
#
# Copyright (c) 2014 Rich Wareham <rjw57@cantab.net>
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
"""ROS node for detecting tips in input images via OpenCV.

"""
import cv2
import rospy
import roslib
import sys
import math
import numpy as np


from tip_detection import image_to_array, TipDetector, reduce_size

from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# Import the ROS message type(s) we'll be using.
from sensor_msgs.msg import Image
from tip_detection_msgs.msg import Tips, Tip

# Default locations for haarcascade data
#DEFAULT_HAAR_CASCADE_DIR = '/usr/share/opencv/haarcascades/'

class TipDetectionNode(object):
    """A node which publishes tip detection results.

    """
    def __init__(self, tips_pub):
        self._tips_pub = tips_pub

        self._latest_image = None

        self._detect_timer = rospy.Timer(rospy.Duration(0.03),
                self._detect_tips_callback)

    def new_input_image(self, image):
        """Called when there is a new input image."""

        # Don't try to do anything if this node is shutting down
        if rospy.is_shutdown():
            return

        # Record this message as the latest image to arrive
        self._latest_image = image

    def _detect_tips_callback(self, event):
        """Called periodically to detect tips in an image and publish the
        result.

        """

        # Get the latest image to arrive
        image = self._latest_image

        # Don't do anything if there is no latest image
        if image is None:
            return

        # "Claim" this image by clearing _latest_image
        self._latest_image = None

        # Create the tip message we will eventually publish
        tips_msg = Tips()

        # Copy the header from the input image so we know when and what these
        # tip detection results relate to.
        tips_msg.header = image.header

        # Parse image message into numpy array
        image_array = image_to_array(image)
        rospy.logdebug('Successfully parsed new image with shape: %s',
            image_array.shape)
        imageArray = reduce_size(image_array,'rgb',2);
	#imageArray = reduce_size(imageArray,'rgb',2);
	#imageArray = reduce_size(imageArray,'rgb',2);
        
        # Detect tips
        detect_tip = TipDetector
        tips = detect_tip(imageArray)

        # Create a tip bounding box for each tip
        #for tip_idx, tip_bbox in enumerate(tips):
        #    rospy.logdebug('Tip #%s at %s', tip_idx+1, tip_bbox)

            # Create tip message
        tip_msg = Tip()
        tip_msg.header = tips_msg.header

            # Initialise roi
	if tips != (0,0):
            tip_msg.roi.x_offset = tips[0]
            tip_msg.roi.y_offset = tips[1]
            tip_msg.roi.do_rectify = False

            # Append to list of tips
        tips_msg.tips.append(tip_msg)

        # Publish tips messages
        self._tips_pub.publish(tips_msg)

	rospy.logdebug('Coordinates of tip: %s',
            tips)

def main():
    """Entry point for node."""

    # Register ourselves as a node with ROS
    rospy.init_node('tip_detect', log_level=rospy.DEBUG)

    # Create the tip detector
    #image = CvBridge.imgmsg_to_cv2(msg)
    #detect_tips = TipDetector()

    # Create a publisher for detected tip results.
    tips_pub = rospy.Publisher(rospy.get_name() + '/tips', Tips,
            queue_size=1)

    # Create an object encapsulating this node's logic
    node = TipDetectionNode(tips_pub)

    # Subscribe to incoming camera images. Note that we set the queue size to
    # 1. This means we automatically drop images if we can't detect tips fast
    # enough.
    rospy.Subscriber('camera/image_raw', Image, node.new_input_image,
            queue_size=1)

    # Run the event loop. Only returns once the node has shutdown.
    rospy.spin()

# Boilerplate to run main() when this file is run.
if __name__ == '__main__':
    main()
