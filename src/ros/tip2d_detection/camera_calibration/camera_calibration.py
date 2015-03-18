# This program calibrates the cameras based on the chessboard image files.
# It will return necessary matrices (rectification and undistortion) in
# order to rectify images fed from the stereo cameras. Images need to be
# rectified to produce the disparity map

import cv2
import numpy as np

# Load the chessboard's corner coordinates from image files
image_points = np.load('src/ros/tip2d_detection/camera_calibration/calibration_data/image_points.npy', mmap_mode='r')

# World coordinates for the corners of a 9x6 chessboard. The width of each
# square is set to be 1.3cm
pattern_size = (9,6)
corner_coordinates = np.zeros((np.prod(pattern_size), 3), np.float32)
corner_coordinates[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
corner_coordinates *= 1.3

# List of world coordinates of the chessboard. This will be used for
# calibration with the chessboard coordinates obtained from chessboard
# image files
object_points = []
n=5 # Number of chessboard images
for i in range(1,n):
    object_points.append(corner_coordinates)

# Size of the camera images
image_size = (640,480)

# Calibrate the cameras
(rms, camera_matrix, dist_coefs, rvecs, tvecs) = cv2.calibrateCamera(object_points, image_points, 
                                                                     image_size)

# Save the undistortion and rectification transformation map
np.save('src/ros/tip2d_detection/camera_calibration/calibration_data/camera_matrix.npy', camera_matrix)
