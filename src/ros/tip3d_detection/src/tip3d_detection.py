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
"""Common utility functions used by the tip_detection package.

"""
import os

import cv2
import numpy as np
import rospy
import math

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

def reduce_size(A,tp,s):
    m = np.shape(A)[0]; n = np.shape(A)[1];
      
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

# Return the coordinates detected the most often (for stability purposes) 
def count(a):
    results = []
    count = []
    final=[]
    for x in a:
        if x not in results and x!=(0,0):
            results.append(x)
            count.append(0)
    for x in a:
        for i in range(0,len(results)):
            if x==results[i]:
                count[i]+=1
    for i in range(0,len(results)):
        if count[i]==max(count):
            final.append(results[i])	
    return max(final)

def TipDetector(I):
    # Detect tips in an image

    I.flags.writeable = True

    K=np.zeros((np.shape(I)[0],np.shape(I)[1],3),np.uint8)*255
    gray = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    canny=cv2.Canny(gray,50,100)
    contours,hier = cv2.findContours(canny,1,2)

    n=3

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.04*cv2.arcLength(cnt,True),True)
        if len(approx)==n:
            cv2.drawContours(K,[cnt],0,(0,255,0),1)

    gray1 = cv2.cvtColor(K, cv2.COLOR_BGR2GRAY)
    canny1=cv2.Canny(gray1,50,100)
    gaussian_blur1 = cv2.GaussianBlur(gray1,(7,7),0)
    canny_blur1 = cv2.Canny(gaussian_blur1,50,110)
    contours1,hier1 = cv2.findContours(canny_blur1,1,2)

    centre=[]
    
    for cnt1 in contours1:
        approx1 = cv2.approxPolyDP(cnt1,0.04*cv2.arcLength(cnt1,True),True)
        if len(approx1)==n:
            a=(approx1[0][0][0],approx1[0][0][1])
            b=(approx1[1][0][0],approx1[1][0][1])
            c=(approx1[2][0][0],approx1[2][0][1])
            ab=(a[0]-b[0])**2+(a[1]-b[1])**2
            ac=(a[0]-c[0])**2+(a[1]-c[1])**2
            bc=(c[0]-b[0])**2+(c[1]-b[1])**2
            if ((ab + ac - bc) / (2 *math.sqrt(ab*ac))) < 0.9 and ((bc + ac - ab) / (2 *math.sqrt(bc*ac))) < 0.9 and ((ab + bc - ac) / (2 *math.sqrt(ab*bc))) < 0.9:            
                if min(ab,ac,bc)==ab and ab>50:
                    centre.append(c)
                elif min(ab,ac,bc)==ac and ac>50:
                    centre.append(b)
                elif min(ab,ac,bc)==bc and bc>50:
                    centre.append(a)
    
    if not centre:
        return (0,0)
    else:
        return count(centre)   
        
def world_coordinates(u,v,left,right):
    # Load the undistortion and rectification transformation map
    path = '/home/ros/workspace/src/robotic_surgery/tip3d_detection/camera_calibration/calibration_data/'
    left_undistortion = np.load(path + 'left_undistortion.npy', mmap_mode='r')
    left_rectification = np.load(path + 'left_rectification.npy', mmap_mode='r')
    right_undistortion = np.load(path + 'right_undistortion.npy', mmap_mode='r')
    right_rectification = np.load(path + 'right_rectification.npy', mmap_mode='r')

    # Rectify left and right images
    left_rectified = cv2.remap(left, left_undistortion, left_rectification, 
	                           cv2.INTER_NEAREST)
    right_rectified = cv2.remap(right, right_undistortion, right_rectification, 
	                    		cv2.INTER_NEAREST)

    # Specify parameters for the semi global block matching algorithm
    stereo = cv2.StereoSGBM(minDisparity=16, numDisparities=96, SADWindowSize=3, 
	                		P1=216, P2=864, disp12MaxDiff=14, preFilterCap=100, 
			                uniquenessRatio=15, speckleWindowSize=150, 
			                speckleRange=1, fullDP=False)

    # Compute the disparity map
    disparity = stereo.compute(left_rectified, right_rectified)

    # Adjust the disparity map for clarity in depth
    disparity = disparity.astype(np.float32) / 16.0 

    # Disparity-to-depth mapping matrix
    Q = np.float32([[1, 0, 0, -0.5 * 640],
                    [0, -1, 0, 0.5 * 480],
                    [0, 0, 0, -0.8*640],
                    [0, 0, 1, 0]])

    # Compute the 3D world coordinates
    Image = cv2.reprojectImageTo3D(disparity, Q)

    # Identify the world coordinates of a certain point with left image
    # coordinates (u,v)
    camera_coord = Image[u,v]
    
    # Convert camera world coordinates to robot world coordinates
    robot_coord = [0.,0.,0.]
    distance_between_cameras = 9.1
    accurate_vertical_distance = 18
    vertical_distance_from_camera_to_the_board = 29.8
    tilted_distance_from_camera_to_the_board = 31.3
    centre_shift_to_right = 3.7
    centre_shift_forward = 73.8
    angle = math.acos(vertical_distance_from_camera_to_the_board/tilted_distance_from_camera_to_the_board)
    robot_coord[0] = (camera_coord[1]*math.sin(angle) - camera_coord[2]*math.cos(angle) + centre_shift_forward)/100
    robot_coord[1] = (camera_coord[0] - centre_shift_to_right)/100
    robot_coord[2] = (-(camera_coord[1]*math.cos(angle) + camera_coord[2]*math.sin(angle)) + accurate_vertical_distance)/100
    
    return robot_coord
