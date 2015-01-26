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
"""ROS node for drawing boxes over point detection results.

"""
import itertools

import cv2
import rospy

from point_detection import image_to_array, array_to_image

# Import the ROS message type(s) we'll be using.
from sensor_msgs.msg import Image
from point_detection_msgs.msg import Points, Point

class PointHighlighter(object):
    """An object which is passed incoming images and point detection results and
    publishes an image when one or other matches on sequence number.

    The object maintains a small buffer of incoming images and results in order
    to match up detection results with images. This is probably overkill.

    *publisher* is a ROS publisher to which the outgoing image should be written.

    """
    def __init__(self, publisher):
        self._image_pub = publisher

        # Buffers for incoming images and points. The first item in the list is
        # the most recently received message.
        self._image_buffer = []
        self._points_buffer = []

        # The sequence number of the last drawn image to avoid double-draw
        self._last_published_seq = None

    def new_image(self, image):
        """Call this when there is a new incoming image."""
        self._image_buffer = [image,] + self._image_buffer[:10]
        self._check_for_match()

    def new_points(self, points):
        """Call this when there are some new incoming points."""
        self._points_buffer = [points,] + self._points_buffer[:2]
        self._check_for_match()

    def _check_for_match(self):
        """Called when a new image or points result has arrived. Look for the
        latest match based on sequence number.

        """
        # This uses some cool Python iterator magic :)
        all_pairs = itertools.product(self._image_buffer, self._points_buffer)
        matches = list(
            (image, points)
            for image, points in all_pairs
            if image.header.stamp == points.header.stamp
        )
        if len(matches) == 0:
            return

        # Sort matches by image sequence number
        matches.sort(key=lambda m: m[0].header.seq, reverse=True)

        # Don't publish if we already have
        image, points = matches[0]
        if image.header.seq == self._last_published_seq:
            return

        # We have a match, publish a combined image
        self._draw_points(image, points)

    def _draw_points(self, image, points):
        """Take an Image message and a Points message, draw a boc around the
        points in the image and publish the result.

        Record *image*'s sequence number in _last_published_seq.
        """

        # Convert image into RGB image
        rgb_im = image_to_array(image)

        # Use OpenCV to draw boxes in image
        for point in points.points:
            roi = point.roi

            # NB: cv2.circle will modify the image it works on.
	    if (roi.x_offset, roi.y_offset)!=(0,0):
        	cv2.circle(rgb_im,
                    (roi.x_offset, roi.y_offset),
                    5,
                    (255, 0, 0), # green
                    2 # thickness
                )

        # Publish image
        self._image_pub.publish(array_to_image(rgb_im))
        self._last_published_seq = image.header.seq

def main():
    """Entry point for node."""

    # Register ourselves as a node
    rospy.init_node('highlight_points')

    # Create a publisher for the highlighted camera image
    image_pub = rospy.Publisher(rospy.get_name() + '/image_raw', Image,
            queue_size=1)

    # Create the object which does the bulk of the work
    point_higlighter = PointHighlighter(image_pub)

    # Create subscribers to the incoming images and incoming point detection
    # results.
    rospy.Subscriber('camera/image_raw', Image, point_higlighter.new_image,
            queue_size=1)
    rospy.Subscriber('point_detect/points', Points, point_higlighter.new_points,
            queue_size=1)

    # Run the event loop. Only returns once the node has shutdown.
    rospy.spin()

# Boilerplate to run main() when this file is run.
if __name__ == '__main__':
    main()
