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

    # Find orange

    Y=0.3*I[:,:,2]+0.6*I[:,:,1]+0.1*I[:,:,0]
    V=0.4375*I[:,:,2]-0.375*I[:,:,1]-0.0625*I[:,:,0]
    U=-0.15*I[:,:,2]-0.3*I[:,:,1]+0.45*I[:,:,0]

    M=np.ones((np.shape(I)[0], np.shape(I)[1]), np.uint8)*255
    for i in range(0,np.shape(I)[0]):
        for j in range(0,np.shape(I)[1]):
            if V[i,j]>15 and U[i,j]>-7:
                M[i,j]=0
    kernel = np.ones((5,5),np.uint8)   
    M = cv2.morphologyEx(M, cv2.MORPH_OPEN, kernel)
    M=cv2.GaussianBlur(M,(7,7),8)

    # find Harris corners
    dst = cv2.cornerHarris(M,5,3,0.04)
    dst = cv2.dilate(dst,None)
    ret, dst = cv2.threshold(dst,0.7*dst.max(),255,0)
    dst = np.uint8(dst)

    gray1 = cv2.cvtColor(I,cv2.COLOR_BGR2GRAY)
    gray1 = np.float32(gray1)
    dst1 = cv2.cornerHarris(gray1,3,3,0.04)
    dst1 = cv2.dilate(dst1,None)
    ret1, dst1 = cv2.threshold(dst1,0.01*dst1.max(),255,0)
    dst1 = np.uint8(dst1)
    E1 = np.where(dst1 > 0.01*dst1.max())
	
    # identify the tip
    E = np.where(dst > 0.01*dst.max())
    rospy.logdebug('Shape of E: %s', np.shape(E))
    rospy.logdebug('Shape of E1: %s', np.shape(E1))
    if not E or not E1:
        return 
    D=[]
    ind1 = np.lexsort((E1[1],E1[0]))
    C1=[(E1[1][i],E1[0][i]) for i in ind1]
    ind = np.lexsort((E[1],E[0]))
    C=[(E[1][i],E[0][i]) for i in ind]
    for i in range(1,np.shape(C1)[0]):
        for j in range(1,np.shape(C)[0]):
       	    if abs(C1[i][0]-C[j][0])<5 and abs(C1[i][1]-C[j][1])<5:
		D.append([int(np.uint(C1[i][0]*2)), int(np.uint(C1[i][1]*2))])
    if not D:
	return [0,0]
    else:
        return count(D)
        
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
    robot_coord[0] = (camera_coord[1]*math.cos(angle) - camera_coord[2]*math.sin(angle) + centre_shift_forward)/100
    robot_coord[1] = (camera_coord[0] + centre_shift_to_right)/100
    robot_coord[2] = (-(camera_coord[1]*math.sin(angle) + camera_coord[2]*math.cos(angle)) + accurate_vertical_distance)/100
    
    return robot_coord
