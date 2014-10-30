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
#
"""Common utility functions used by the face_detection package.

"""
import os

import cv2
import numpy as np
import rospy

# Import the ROS message type(s) we'll be using.
from sensor_msgs.msg import Image

def image_to_array(image):
    """Parse a ROS image topic into a NumPy array. Raises ValueError if the
    image is in an unsupported encoding.

    """
    # See http://docs.ros.org/api/sensor_msgs/html/msg/Image.html for the
    # fields encoded in image.

    # TODO: support more image encodings
    if image.encoding != 'rgb8':
        rospy.logerr(
            'Image encoding "%s" not supported. '
            'Image must be in "rgb8" encoding.', image.encoding
        )
        raise ValueError('Image encoding must be "rgb8".')

    # We know the image is in rgb8 encoding. This is a group of three bytes
    # encoding the red, green and blue image channels. Use numpy to parse the
    # image into a height x width x 3 array.
    image_data = np.frombuffer(image.data, dtype=np.uint8)

    # Reshape image data into "height" rows of "step" bytes
    image_data = image_data.reshape((image.height, image.step), order='C')

    # Trim image data to have correct width. (Note that "step" may not
    # correspond to "widths" * 3 bytes.)
    image_data = image_data[:, :(image.width * 3)]

    # Reshape image data to have r, g and b bytes on a new axis
    return image_data.reshape((image.height, image.width, 3), order='C')

def array_to_image(array):
    """Takes a NxMx3 array and converts it into a ROS image message.

    """
    # Sanity check the input array shape
    if len(array.shape) != 3 or array.shape[2] != 3:
        raise ValueError('Array must have shape MxNx3')

    # Ravel the array into a single buffer
    image_data = (array.astype(np.uint8)).tostring(order='C')

    # Create the image message
    image_msg = Image()
    image_msg.height = array.shape[0]
    image_msg.width = array.shape[1]
    image_msg.encoding = 'rgb8'
    image_msg.is_bigendian = 0
    image_msg.step = array.shape[1] * 3
    image_msg.data = image_data

    return image_msg

class FaceDetector(object):
    """Detect faces in an image using OpenCV.

    *haar_cascade_dir* should be the path to a directory containing the
    classifier data shipped with OpenCV.
    """

    # NB: much of the code in this class is taken from the OpenCV
    # "py_face_detection" tutorial.

    def __init__(self, haar_cascade_dir):
        if not os.path.isdir(haar_cascade_dir):
            raise IOError('Haar cascade directory "{0}" does not exist'.format(
                haar_cascade_dir))

        # Set internal attributes
        self._haar_cascade_dir = haar_cascade_dir

        # Create OpenCV classifiers
        ff_classifier_fn = os.path.join(self._haar_cascade_dir,
                'haarcascade_frontalface_default.xml')
        if not os.path.isfile(ff_classifier_fn):
            raise IOError('Haar cascade classifier "{0}" does not exist'.format(
                ff_classifier_fn))
        self._ff_classifier = cv2.CascadeClassifier(ff_classifier_fn)

    def detect(self, image):
        """Detect faces. *image* should be a NumPy array containing a grayscale
        image.

        Returns a sequence of 4-tuples giving the x- and y-co-ordinates of the
        top-left of the face's bounding box and the width and height. all
        values are in pixels.
        """
        return self._ff_classifier.detectMultiScale(image, 1.3, 5)

    def __call__(self, image):
        """Convenience method which allows us to treat the face detector as a
        callable.

        """
        return self.detect(image)
