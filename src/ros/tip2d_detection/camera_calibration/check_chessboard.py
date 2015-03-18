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
n=5;

# The coordinates of the chessboard corners taken by the left and right camera 
# will be stored here
image_points=[]

a=0

for i in range(1,n):

    # read left images
    if i<10:
        image = cv2.imread('src/ros/tip2d_detection/camera_calibration/calibration_pictures/image_0'+repr(i)+'.jpg')
    else:
        image = cv2.imread('src/ros/tip2d_detection/camera_calibration/calibration_pictures/image_'+repr(i)+'.jpg')

    # convert images to grayscale      
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # identify and save chessboard's corners' coordinates
    ret1, corners1 = cv2.findChessboardCorners(gray,(9, 6))
    image_points.append(corners1.reshape(-1,2))

    # print out the index of unacceptable files
    if not ret1:
        a=1
        print i
        
if a==1:
    print "The numbers listed above are indexes of faulty images"
    print "Remove faulty pair of images (even if only 1 in the pair is faulty)"
    print "Make sure the indexes still start from 01 to 05"
    print "Rerun this script"
else:
    print "Everything is good"

# Save chessboard's corners' coordinates
np.save('src/ros/tip2d_detection/camera_calibration/calibration_data/image_points.npy', image_points)
