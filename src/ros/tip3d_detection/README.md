# 3D tip detection

This program detects a tip and returns its 3D world coordinates

# Structure

tip3d_detection folder contains:
(please ignore files that are not mentioned here, they might be used later on to assist
in debugging or to make this program easier to use but they have not been fully implemented)

1) camera_calibration folder
    +) calibration_data: an empty folder where the calibration data will be created
    +) calibration_pictures: 50 pairs of .ppm left & right images (taken by left
    and right camera). There is a chessboard in all images but at different 
    orientations. They will be used to calibrate the cameras
    +) check_chessboard.py: read the 50 pairs of images, check if they are acceptable
    and create .npy files containing chessboard coordinates
    +) camera_calibration.py: calibrate the cameras and create .npy files containing
    the undistortion and rectification transformation map
    
2) launch folder
    +) launch file: launch 2 nodes to view input left and right cameras and a node
    for tip localisation.
    
3) scripts
    +) localise_tip: main file for 3D tip detection
    
4) src
    +) tip3d_detection.py: contains functions with all the algorithms. The most important
    ones are: TipDetector (use left camera to detect tip and return 2D image coordinates),
    and world_coordinates (convert the 2D image coordinates to robot's 3D world coordinates)
    
# Usage

STEP 1: Check the paths for the cameras. Left camera is dev/video0, right is dev/video1.
If the paths are different, either adjust the launch file accordingly or disable any built in
cameras (eg. laptop's camera), plug the left camera before plugging the right one.

STEP 2: Workspace should be 'robotic_surgery'. 'make build' to ensure that you have openCV. From terminal, run the following:

python ../check_chessboard.py

If it prints out "Everything is good" then proceed to STEP 2, otherwise follow the instructions
in the print out messages. There should not be any error messages.

STEP 3: From terminal, run the following:

python ../camera_calibration.py

This should take some time (varies depending on the position and orientation of the cameras,
roughly 2-5 min)

STEP 1-3 needs to be done only once to get the calibration files. The calibration is completed using existing images rather than real time images.

STEP 4: From terminal, run the following:

make delete_all_my_work

make build gui_launch PKG="tip3d_detection" LAUNCH=tip3d_detect.launch

To view the published 3D coordinates, in a new ros terminal, type:

rostopic echo /localise_tip/tip

(if it runs error, run "rostopic list" to check the exact naming for the topic)
