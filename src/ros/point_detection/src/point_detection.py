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
#
"""Common utility functions used by the point_detection package.

"""
import os

import cv2
import numpy as np
import rospy
import math
from scipy import ndimage

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


def PointDetector(I):
    I.flags.writeable = True

    K=np.zeros((np.shape(I)[0],np.shape(I)[1],3),np.uint8)*255
    gray = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    gaussian_blur = cv2.GaussianBlur(gray,(5,5),0)
    canny_blur = cv2.Canny(gaussian_blur,50,100)
    contours,hier = cv2.findContours(canny_blur,1,2)

    n=4

    for cnt in contours:
	approx = cv2.approxPolyDP(cnt,0.02*cv2.arcLength(cnt,True),True)
        if len(approx)==n:
            cv2.drawContours(K,[cnt],0,(0,255,0),1)

    gray1 = cv2.cvtColor(K, cv2.COLOR_BGR2GRAY)
    gaussian_blur1 = cv2.GaussianBlur(gray1,(5,5),0)
    canny_blur1 = cv2.Canny(gaussian_blur1,50,100)
    contours1,hier1 = cv2.findContours(canny_blur1,1,2)

    tri=[]
    centre=[]
    for cnt1 in contours1:
        approx1 = cv2.approxPolyDP(cnt1,0.02*cv2.arcLength(cnt1,True),True)
        if len(approx1)==n:
            a=(approx1[0][0][0]-approx1[1][0][0])**2+(approx1[0][0][1]-approx1[1][0][1])**2
            b=(approx1[0][0][0]-approx1[2][0][0])**2+(approx1[0][0][1]-approx1[2][0][1])**2
            c=(approx1[0][0][0]-approx1[3][0][0])**2+(approx1[0][0][1]-approx1[3][0][1])**2
            if a > 0.09*b and a > 0.09*c and b > 0.09*a and c > 0.09*a:
                cv2.drawContours(I,[cnt1],0,(0,255,0),2)
                centre_x=0
                centre_y=0
                for i in range(0,len(approx1)):
                    centre_x=centre_x+approx1[i][0][0]/len(approx1)
                    centre_y=centre_y+approx1[i][0][1]/len(approx1)
                centre.append((centre_x,centre_y))
                #tri.append(sort(approx1))

    return centre

def reduce_size(A,tp,s):
    m = np.shape(A)[0]; n = np.shape(A)[1];
    
    # check type of image
    if tp != 'gray' and tp != 'rgb':
        error('This function only accepts two types of images: "gray" and "rgb"');
    
    # check value of s
    if m % s != 0 or n % s != 0:
        error('Image dimension is not divisible by s!')
    
    # reduce gray scale images by sxs
    if tp == 'gray' and s == 2:
        return (
            A[0::2, 0::2]/4 + A[1::2, 0::2]/4 + A[0::2, 1::2]/4 + A[1::2, 1::2]/4
        )
    elif ((tp == 'gray') and (s == 3)):
        return (
            A[0::3, 0::3]/9 + A[1::3, 0::3]/9 + A[2::3, 0::3]/9 + A[0::3, 1::3]/9 + A[1::3, 1::3]/9
            + A[2::3, 1::3]/9 + A[0::3, 2::3]/9 + A[1::3, 2::3]/9 + A[2::3, 2::3]/9
        )
    
    # reduce rgb images by sxs
    if tp == 'rgb' and s == 2:
        return (
            A[0::2, 0::2, :]/4 + A[1::2, 0::2, :]/4 + A[0::2, 1::2, :]/4 + A[1::2, 1::2, :]/4
        )
    if tp == 'rgb' and s == 3:
        return (
            A[0::3, 0::3, :]/9 + A[1::3, 0::3, :]/9 + A[2::3, 0::3, :]/9 + A[0::3, 1::3, :]/9 + A[1::3, 1::3, :]/9
            + A[2::3, 1::3, :]/9 + A[0::3, 2::3, :]/9 + A[1::3, 2::3, :]/9 + A[2::3, 2::3, :]/9
        )

def sort(b):
    less = []
    equal = []
    greater = []
      
    if len(b) > 1:
        pivot = b[0][0][0]
        for i in range(0,len(b)):
            if b[i][0][0] < pivot:
                less.append(b[i])
            if b[i][0][0] == pivot:
                equal.append(b[i])
            if b[i][0][0] > pivot:
                greater.append(b[i])
        return sort(less)+equal+sort(greater)
    else:
        return b    
