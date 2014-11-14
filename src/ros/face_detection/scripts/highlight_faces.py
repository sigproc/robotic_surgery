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
"""ROS node for drawing boxes over face detection results.

"""
import itertools

import cv2
import rospy

from face_detection import image_to_array, array_to_image

# Import the ROS message type(s) we'll be using.
from sensor_msgs.msg import Image
from face_detection_msgs.msg import Faces, Face

class FaceHighlighter(object):
    """An object which is passed incoming images and face detection results and
    publishes an image when one or other matches on sequence number.

    The object maintains a small buffer of incoming images and results in order
    to match up detection results with images. This is probably overkill.

    *publisher* is a ROS publisher to which the outgoing image should be written.

    """
    def __init__(self, publisher):
        self._image_pub = publisher

        # Buffers for incoming images and faces. The first item in the list is
        # the most recently received message.
        self._image_buffer = []
        self._faces_buffer = []

        # The sequence number of the last drawn image to avoid double-draw
        self._last_published_seq = None

    def new_image(self, image):
        """Call this when there is a new incoming image."""
        self._image_buffer = [image,] + self._image_buffer[:10]
        self._check_for_match()

    def new_faces(self, faces):
        """Call this when there are some new incoming faces."""
        self._faces_buffer = [faces,] + self._faces_buffer[:2]
        self._check_for_match()

    def _check_for_match(self):
        """Called when a new image or faces result has arrived. Look for the
        latest match based on sequence number.

        """
        # This uses some cool Python iterator magic :)
        all_pairs = itertools.product(self._image_buffer, self._faces_buffer)
        matches = list(
            (image, faces)
            for image, faces in all_pairs
            if image.header.stamp == faces.header.stamp
        )
        if len(matches) == 0:
            return

        # Sort matches by image sequence number
        matches.sort(key=lambda m: m[0].header.seq, reverse=True)

        # Don't publish if we already have
        image, faces = matches[0]
        if image.header.seq == self._last_published_seq:
            return

        # We have a match, publish a combined image
        self._draw_faces(image, faces)

    def _draw_faces(self, image, faces):
        """Take an Image message and a Faces message, draw a boc around the
        faces in the image and publish the result.

        Record *image*'s sequence number in _last_published_seq.
        """

        # Convert image into RGB image
        rgb_im = image_to_array(image)

        # Use OpenCV to draw boxes in imahe
        for face in faces.faces:
            roi = face.roi

            # NB: cv2.rectangle will modify the image it works on.
            cv2.rectangle(rgb_im,
                    (roi.x_offset, roi.y_offset),
                    (roi.x_offset + roi.width, roi.y_offset + roi.height),
                    (0, 255, 0), # green
                    2, # thickness
            )

        # Publish image
        self._image_pub.publish(array_to_image(rgb_im))
        self._last_published_seq = image.header.seq

def main():
    """Entry point for node."""

    # Register ourselves as a node
    rospy.init_node('highlight_faces')

    # Create a publisher for the highlighted camera image
    image_pub = rospy.Publisher(rospy.get_name() + '/image_raw', Image,
            queue_size=1)

    # Create the object which does the bulk of the work
    face_higlighter = FaceHighlighter(image_pub)

    # Create subscribers to the incoming images and incoming face detection
    # results.
    rospy.Subscriber('camera/image_raw', Image, face_higlighter.new_image,
            queue_size=1)
    rospy.Subscriber('face_detect/faces', Faces, face_higlighter.new_faces,
            queue_size=1)

    # Run the event loop. Only returns once the node has shutdown.
    rospy.spin()

# Boilerplate to run main() when this file is run.
if __name__ == '__main__':
    main()
