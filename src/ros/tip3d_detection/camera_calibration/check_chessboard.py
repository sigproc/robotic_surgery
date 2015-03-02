# This program accesses all the chessboard image files and identifies which one
# is suitable to do the camera calibration. The original raw images are in the
# folder called calibration_pictures and the image file names are in the format
# left_xx.ppm for images taken by left camera and right_xx.ppm for images taken
# by right camera (xx is the index number for the image). This program should be
# rerun once all chessboard image files in the calibration_pictures folder are
# acceptable.

# The chessboard used has 9 rows and 6 columns (number of rows and columns are 
# identified by counting number of inward corners of the squares)

import cv2
import numpy as np

# number of raw image pairs
n=50;

# The coordinates of the chessboard corners taken by the left and right camera 
# will be stored here
left_image_points=[]
right_image_points=[]

print "LEFT"
for i in range(1,n):

    # read left images
    if i<10:
        image = cv2.imread('src/ros/tip3d_detection/camera_calibration/calibration_pictures/left_0'+repr(i)+'.ppm')
    else:
        image = cv2.imread('src/ros/tip3d_detection/camera_calibration/calibration_pictures/left_'+repr(i)+'.ppm')

    # convert images to grayscale      
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # identify and save chessboard's corners' coordinates
    ret1, corners1 = cv2.findChessboardCorners(gray,(9, 6))
    left_image_points.append(corners1.reshape(-1,2))

    # print out the index of unacceptable files
    if not ret1:
        print i
        
print "RIGHT"
for i in range(1,n):

    # read right images
    if i<10:
        image = cv2.imread('src/ros/tip3d_detection/camera_calibration/calibration_pictures/right_0'+repr(i)+'.ppm')
    else:
        image = cv2.imread('src/ros/tip3d_detection/camera_calibration/calibration_pictures/right_'+repr(i)+'.ppm')

    # convert images to grayscale            
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # identify and save chessboard's corners' coordinates
    ret2, corners2 = cv2.findChessboardCorners(gray,(9, 6))
    right_image_points.append(corners2.reshape(-1,2))

    # print out the index of unacceptable files
    if not ret2:
        print i

# Save chessboard's corners' coordinates
np.save('src/ros/tip3d_detection/camera_calibration/calibration_data/left_image_points', left_image_points)
np.save('src/ros/tip3d_detection/camera_calibration/calibration_data/right_image_points', right_image_points)
