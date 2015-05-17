When setting up a new robot model, remember to:
(1) change ($find setup_config) ==> ($find XXpkg_nameXX) in planning_context.launch file
(2) change the XXX.xacro file in the launch file (value of arg)

Note that, when copying from kinematics_smart:
(1) The 'type' for nodes in launch files point to the name of the executable defined in CMakeLists.txt
(2) One row has to be removed from the vector for the static version because there are only 5 joints.

----

#Kinematics

This program takes in a 3D position reference to the base of the robot arm, invokes inverse kinematics (IK) to calculate the required joint angles to reach the desired position, and then publish the joint angles to the robot arm using a 2nd order proportional controller. It also compares (i) the tip position that the robot thinks it is in and (ii) the camera-derived position to invoke a force feedback loop, in order to return the tip back to its intended position.
 
#Structure

kinematics_smart/arm_moveit_generated folder contains:
(please ignore files that are not mentioned here, they came with the initial setup)

1) config folder
    +) Contains the calibration files that were generated using the MoveIt! Setup Assistant. The logical and physical description of the robot arm is stored in a separate folder called 'smart_arm_desc'. Use the 'robot_desc.xacro' or 'smart_arm_desc.xacro' to setup the calibration files.
    +)kinematics.yaml: an important file that defines the configuration of the kinematics solver. The line 'position_only_ik: True' is added so that only the position (and not orientation) of the robot arm is considered for IK. This is important especially for a robot arm with low degrees of freedom. This can be relaxed by removing the line if a robot arm with more joints are used.
    
2) launch folder
    +) Contains all the launch files. 
    +) kmt_test.launch: Launch the configurations of the robot arm, load rViz (a visualisation tool), load IK (IK.cpp), node that publishes a position to IK solver (pos_for_IK.py) and also the 2nd order controller (../smart_arm_controller/prop_closed_2nd_order.py). 
    +) kinematic_model_tutorial.launch: Very similar to kinematic_model_tutorial with rViz. rViz slows the program down. 'IK' node was previously named 'kinematic_model_tutorial'.
    +) integration.launch: Integrated Force Feedback Loop, Camera calibration, IK, 2nd order controller in one launch file
    
3) src folder
    +) Contains important source files for the task
    +) kinematic_model_tutorial.cpp is the earlier version of IK_modified_arm.cpp
    +) pos_for_IK: sends the desired world position to the IK solver every 5 secs. This will be replaced by inputs from camera
    +) IK_modified_arm.py: takes in a given 3D world position and convert it into joint angles. This publishes the joint angles to e.g. /shoulder_pitch_controller/intermediate_command.
    +) desired_pos.py: outputs the desired tip position in world coordinates
    +) force_feedback.py: compares the desired position and camera-derived position, and outputs a new /pos_for_IK for the IK solver to return the tip back to its intended position
    +) make_cobra_pose.py: demo to move the robot around using angular positions of servos
    
#Other dependencies

1. robotic_surgery/src/ros/crustcrawler_smart_arm/smart_arm_controller/scripts/prop_closed_2nd_order.py: Listens to the intermediate commands sent from IK.cpp, and publishes to the e.g. /shoulder_pitch_controller/state using a 2nd order controller if the tip of the robot arm has not reached the desired position. This is done at a significantly higher rate than 5 seconds (which is the time lapse between each camera input).
    
--end--
