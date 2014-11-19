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
"""ROS node for detecting faces in input images via OpenCV.

"""
import cv2
import rospy

from face_detection import image_to_array, FaceDetector

# Import the ROS message type(s) we'll be using.
from sensor_msgs.msg import Image
from face_detection_msgs.msg import Faces, Face

# Default locations for haarcascade data
DEFAULT_HAAR_CASCADE_DIR = '/usr/share/opencv/haarcascades/'

class FaceDetectionNode(object):
    """A node which publishes face detection results.

    """
    def __init__(self, detect_faces, faces_pub):
        self._detect_faces = detect_faces
        self._faces_pub = faces_pub

        # Detecting faces can take a long time. Certainly it takes longer then
        # 1/30 second which is what would be required if we wanted to do it
        # with no frame drop. Consequently we take a slightly subtle approach
        # to processing frames.
        #
        # The code run by a Subscriber callback in ROS *must* be fast. If it is
        # not fast then we will get horrible latency problems as we try to
        # process the backlog of images. (Search "buffer bloat" for some
        # technical discussion of this issue.)
        self._latest_image = None

        # We try to be as fast as possible in new_input_image by simply
        # recording the image in the _latest_image attribute when it arrives
        # and then immediately return from the function. This makes sure that
        # the ROS network stack is speedy, latency is reduced and we avoid the
        # cardinal sin of doing non-trivial amounts of work on the network
        # thread.
        #
        # We do need to do work, though. _detect_timer is a ROS timer which
        # fires once every 0.03 seconds (i.e. ~30Hz). A ROS timer is clever in
        # that it will only fire on the *next* 0.03 second boundary even if the
        # callback takes longer than 0.03 seconds to run. We make use of this to
        # de-couple the detection of faces from the rate of incoming images.
        #
        # The _detect_faces_callback function will attempt to detect faces in
        # _latest_image and then set _latest_image to None. In this way we can
        # have some images arrive on the network while _detect_faces_callback
        # is busy which wont be queued up since only the last image to arrive
        # gets set as _latest_image.
        #
        # Usually this sort of rate limiting needs only be done when one is
        # processing a raw video stream (or "drinking from the firehose" as it
        # is known.)
        #
        # A cleaner solution would be to use a semaphore and some sort of
        # worker thread but this example is supposed to be simple and not an
        # exercise in writing efficient event-driven code. An interesting
        # exercise would be to re-write this using threading.Semaphore().
        self._detect_timer = rospy.Timer(rospy.Duration(0.03),
                self._detect_faces_callback)

    def new_input_image(self, image):
        """Called when there is a new input image."""

        # Don't try to do anything if this node is shutting down
        if rospy.is_shutdown():
            return

        # Record this message as the latest image to arrive
        self._latest_image = image

    def _detect_faces_callback(self, event):
        """Called periodically to detect faces in an image and publish the
        result.

        """
        # Get the latest image to arrive
        image = self._latest_image

        # Don't do anything if there is no latest image
        if image is None:
            return

        # "Claim" this image by clearing _latest_image
        self._latest_image = None

        # Create the face message we will eventually publish
        faces_msg = Faces()

        # Copy the header from the input image so we know when and what these
        # face detection results relate to.
        faces_msg.header = image.header

        # Parse image message into numpy array
        image_array = image_to_array(image)
        rospy.logdebug('Successfully parsed new image with shape: %s',
            image_array.shape)

        # Convert image to grayscale
        gray_im = cv2.cvtColor(image_array, cv2.COLOR_RGB2GRAY)

        # Detect faces
        faces = self._detect_faces(gray_im)

        # Create a face bounding box for each face
        for face_idx, face_bbox in enumerate(faces):
            rospy.logdebug('Face #%s at %s', face_idx+1, face_bbox)

            # Create face message
            face_msg = Face()
            face_msg.header = faces_msg.header

            # Initialise roi
            face_msg.roi.x_offset = face_bbox[0]
            face_msg.roi.y_offset = face_bbox[1]
            face_msg.roi.width = face_bbox[2]
            face_msg.roi.height = face_bbox[3]
            face_msg.roi.do_rectify = False

            # Append to list of faces
            faces_msg.faces.append(face_msg)

        # Publish faces messages
        self._faces_pub.publish(faces_msg)

def main():
    """Entry point for node."""

    # Register ourselves as a node with ROS
    rospy.init_node('face_detect')

    # Create the face detector
    hcd = rospy.get_param('haar_cascade_dir', DEFAULT_HAAR_CASCADE_DIR)
    rospy.logdebug('Creating face detector using data in: %s', hcd)
    detect_faces = FaceDetector(hcd)

    # Create a publisher for detected face results. See [1] for discussion of
    # queue_size parameter.
    # [1] http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    faces_pub = rospy.Publisher(rospy.get_name() + '/faces', Faces,
            queue_size=1)

    # Create a object encapsulating this node's logic
    node = FaceDetectionNode(detect_faces, faces_pub)

    # Subscribe to incoming camera images. Note that we set the queue size to
    # 1. This means we automatically drop images if we can't detect faces fast
    # enough.
    rospy.Subscriber('camera/image_raw', Image, node.new_input_image,
            queue_size=1)

    # Run the event loop. Only returns once the node has shutdown.
    rospy.spin()

# Boilerplate to run main() when this file is run.
if __name__ == '__main__':
    main()
