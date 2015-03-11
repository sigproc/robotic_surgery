#Kinematics

This program takes in a 3D position reference to the base of the robot arm, invokes inverse kinematics (IK) to calculate the required joint angles to reach the desired position, and then publish the joint angles to the robot arm using a 2nd order proportional controller.

#Structure

kinematics_smart/arm_moveit_generated folder contains:
(please ignore files that are not mentioned here, they came with the initial setup)

1) config folder
    +) Contains the calibration files that were generated using the MoveIt! Setup Assistant. The logical and physical description of the robot arm is stored in a separate folder called 'smart_arm_desc'. Use the 'robot_desc.xacro' or 'smart_arm_desc.xacro' to setup the calibration files.
    +)kinematics.yaml: an important file that defines the configuration of the kinematics solver. The line 'position_only_ik: True' is added so that only the position (and not orientation) of the robot arm is considered for IK. This is important especially for a robot arm with low degree of freedom. This can be relaxed by removing the line if a robot arm with more joints are used.
    
2) launch folder
    +) Contains all the launch files. 
    +) kmt_test.launch: Launch the configurations of the robot arm, load rViz (a visualisation tool), load IK (IK.cpp), node that publishes a position to IK solver (pos_for_IK.py) and also the 2nd order controller (../smart_arm_controller/prop_closed_2nd_order.py). 
    +) kinematic_model_tutorial.launch: Very similar to kinematic_model_tutorial with rViz. rViz slows the program down. 'IK' node was previously named 'kinematic_model_tutorial'.
    
3) src folder
    +) Contains important source files for the task
    +) kinematic_model_tutorial.cpp is the earlier version of IK.cpp
    +) pos_for_IK: sends the desired world position to the IK solver every 5 secs. This will be replaced by inputs from camera
    +) IK: takes in a given 3D world position and convert it into joint angles. This publishes the joint angles to e.g. /shoulder_pitch_controller/intermediate_command.
    
#Other dependencies

1. robotic_surgery/src/ros/crustcrawler_smart_arm/smart_arm_controller/scripts/prop_closed_2nd_order.py: Listens to the intermediate commands sent from IK.cpp, and publishes to the e.g. /shoulder_pitch_controller/state using a 2nd order controller if the tip of the robot arm has not reached the desired position. This is done at a significantly higher rate than 5 seconds (which is the time lapse between each camera input).
    
    
    
    
    
(to be continued)...
    
