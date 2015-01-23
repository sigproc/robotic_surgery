#!/usr/bin/env python
#
# The MIT License (MIT)
#
# Copyright (c) 2014 Duong Le <tdl28@cam.ac.uk>
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
"""ROS node for detecting points in input images via OpenCV.

"""
import cv2
import rospy
import roslib
import sys
import math
import numpy as np


from point_detection import image_to_array, PointDetector, reduce_size

from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# Import the ROS message type(s) we'll be using.
from sensor_msgs.msg import Image
from point_detection_msgs.msg import Points, Point

# Default locations for haarcascade data
#DEFAULT_HAAR_CASCADE_DIR = '/usr/share/opencv/haarcascades/'

class PointDetectionNode(object):
    """A node which publishes point detection results.

    """
    def __init__(self, points_pub):
        self._points_pub = points_pub

        self._latest_image = None

        self._detect_timer = rospy.Timer(rospy.Duration(0.03),
                self._detect_points_callback)

    def new_input_image(self, image):
        """Called when there is a new input image."""

        # Don't try to do anything if this node is shutting down
        if rospy.is_shutdown():
            return

        # Record this message as the latest image to arrive
        self._latest_image = image

    def _detect_points_callback(self, event):
        """Called periodically to detect points in an image and publish the
        result.

        """

        # Get the latest image to arrive
        image = self._latest_image

        # Don't do anything if there is no latest image
        if image is None:
            return

        # "Claim" this image by clearing _latest_image
        self._latest_image = None

        # Create the point message we will eventually publish
        points_msg = Points()

        # Copy the header from the input image so we know when and what these
        # point detection results relate to.
        points_msg.header = image.header

        # Parse image message into numpy array
        image_array = image_to_array(image)
        rospy.logdebug('Successfully parsed new image with shape: %s',
            image_array.shape)
        imageArray = reduce_size(image_array,'rgb',2);
	#imageArray = reduce_size(imageArray,'rgb',2);
	#imageArray = reduce_size(imageArray,'rgb',2);
        
        # Detect points
        detect_point = PointDetector
        points = detect_point(image_array)

        # Create a point bounding circle for each point
        for point_idx, point_circle in enumerate(points):
            rospy.logdebug('Point #%s at %s', point_idx+1, point_circle)

            # Create point message
            point_msg = Point()
            point_msg.header = points_msg.header

            # Initialise roi
	    if points != (0,0):
                point_msg.roi.x_offset = point_circle[0]
                point_msg.roi.y_offset = point_circle[1]
                point_msg.roi.do_rectify = False

            # Append to list of points
            points_msg.points.append(point_msg)

        # Publish points messages
        self._points_pub.publish(points_msg)

	rospy.logdebug('Coordinates of point: %s',
            points)

def main():
    """Entry point for node."""

    # Register ourselves as a node with ROS
    rospy.init_node('point_detect', log_level=rospy.DEBUG)

    # Create the point detector
    #image = CvBridge.imgmsg_to_cv2(msg)
    #detect_points = PointDetector()

    # Create a publisher for detected point results.
    points_pub = rospy.Publisher(rospy.get_name() + '/points', Points,
            queue_size=1)

    # Create an object encapsulating this node's logic
    node = PointDetectionNode(points_pub)

    # Subscribe to incoming camera images. Note that we set the queue size to
    # 1. This means we automatically drop images if we can't detect points fast
    # enough.
    rospy.Subscriber('camera/image_raw', Image, node.new_input_image,
            queue_size=1)

    # Run the event loop. Only returns once the node has shutdown.
    rospy.spin()

# Boilerplate to run main() when this file is run.
if __name__ == '__main__':
    main()
