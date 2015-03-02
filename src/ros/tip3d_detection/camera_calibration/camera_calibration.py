# This program calibrates the cameras based on the chessboard image files.
# It will return necessary matrices (rectification and undistortion) in
# order to rectify images fed from the stereo cameras. Images need to be
# rectified to produce the disparity map

import cv2
import numpy as np

# Load the chessboard's corner coordinates from image files
left_image_points = np.load('src/ros/tip3d_detection/camera_calibration/calibration_data/left_image_points.npy', mmap_mode='r')
right_image_points = np.load('src/ros/tip3d_detection/camera_calibration/calibration_data/right_image_points.npy', mmap_mode='r')

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
n=50 # Number of chessboard image pairs
for i in range(1,n):
    object_points.append(corner_coordinates)

# Size of the camera images
image_size = (640,480)

# Set criteria and flags for camera calibration
criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
flags = (cv2.CALIB_FIX_ASPECT_RATIO + cv2.CALIB_ZERO_TANGENT_DIST + 
         cv2.CALIB_SAME_FOCAL_LENGTH)

# Calibrate the cameras
(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, 
 F) = cv2.stereoCalibrate(object_points, left_image_points, 
                          right_image_points, image_size, 
			  criteria=criteria, flags=flags)[1:]

# Compute the rectification transforms for each head of the stereo camera pair
(left_rect_trans, right_rect_trans, left_proj_mats, right_proj_mats, Q, 
 left_valid_boxes, right_valid_boxes) = cv2.stereoRectify(cameraMatrix1, 
							  distCoeffs1,
							  cameraMatrix2, 
							  distCoeffs2, 
							  image_size, R, T, 
							  flags=0)

# Compute the undistortion and rectification transformation map
(left_undistortion, left_rectification) = cv2.initUndistortRectifyMap(cameraMatrix1, 
								      distCoeffs1,
								      left_rect_trans,
								      left_proj_mats, 
                                                                      image_size, 
								      cv2.CV_32FC1)
(right_undistortion, right_rectification) = cv2.initUndistortRectifyMap(cameraMatrix2, 
									distCoeffs2,
									right_rect_trans,
                                                                        right_proj_mats, 
									image_size, 
									cv2.CV_32FC1)

# Save the undistortion and rectification transformation map
np.save('src/ros/tip3d_detection/camera_calibration/calibration_data/left_undistortion', left_undistortion)
np.save('src/ros/tip3d_detection/camera_calibration/calibration_data/left_rectification', left_rectification)
np.save('src/ros/tip3d_detection/camera_calibration/calibration_data/right_undistortion', right_undistortion)
np.save('src/ros/tip3d_detection/camera_calibration/calibration_data/right_rectification', right_rectification)
