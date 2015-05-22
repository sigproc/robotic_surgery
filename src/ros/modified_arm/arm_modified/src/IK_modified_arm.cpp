#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <math.h>
#include <cmath>

void print_kinematic_state(robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup* joint_model_group) {
    // Get joint names
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    // Get Joint Values
    std::vector<double> joint_values;

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
}

std::vector<double> get_kinematic_state(robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup* joint_model_group) {
    // Get joint names
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    // Get Joint Values
    std::vector<double> joint_values;

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    
    return joint_values;
}

bool calcIKSolutionForEffectorPos(Eigen::Vector3d pos, std::vector<double> &output)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("Arm");
   
    /* Enforce the joint limits for this state and check again*/
    kinematic_state->enforceBounds();
    
    Eigen::VectorXd joints(5);
    //Initialise for Cobra Pose
    joints << 0, -1.9722, 0, 0, 0;

    //Forward Kinematics
    kinematic_state->setJointGroupPositions(joint_model_group, joints);
    Eigen::Affine3d end_effector_state;
    end_effector_state = kinematic_state->getGlobalLinkTransform("effector_tip_link");
    
    Eigen::Vector3d cobra_v;

    //This is the eef 3D position in space for cobra pose
    //'Cobra pose' is used as the reference position to be subtracted to identify a point
    //in 3D space. This is because end_effector_state.translate() is relative to eef not base.
    cobra_v << 0.225307, 0.0, 0.210856;
      
    Eigen::Affine3d frame_transform;
    frame_transform = kinematic_state->getFrameTransform("effector_tip_link");
    
    //Convert Affine to Matrix
    Eigen::Matrix3d frame_transform_Matrix;
    frame_transform_Matrix = frame_transform.linear();
    frame_transform_Matrix = frame_transform.matrix().topLeftCorner<3,3>();
    
    //Vector to transform eef position
    Eigen::Vector3d pos_prime;
    Eigen::Vector3d cobra_v_prime;
    pos_prime = frame_transform_Matrix.inverse()*pos;
    cobra_v_prime = frame_transform_Matrix.inverse()*(-cobra_v);
    
    //Remove the cobra_pose component so I can specify 3D space directly
    end_effector_state.translate(cobra_v_prime);
    end_effector_state.translate(pos_prime);
    
    //Inverse Kinematics
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

    if (found_ik)
    {
        output = get_kinematic_state(kinematic_state, joint_model_group);
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }

    return found_ik;
}

std::vector<ros::Publisher> joint_publishers;

void gotPosition(const std_msgs::Float64MultiArray& msg)
{
    // Pull out three components of position into an Eigen vector
    Eigen::Vector3d pos(msg.data[0], msg.data[1], msg.data[2]);
    
    // Run IK
    std::vector<double> ik_solution;
    bool found_ik = calcIKSolutionForEffectorPos(pos, ik_solution);
    
    // If we found one, publish it.
    if(found_ik) {
        for(int i=0; i<4; ++i) {
            std_msgs::Float64 msg; 
            msg.data = ik_solution[i];
            joint_publishers[i].publish(msg);
        }
    }
    /* //Tested this to improve IK success rate, but realised that there is in built IK iteration
    // in ./config/kinematics
    // Otherwise, iterate 2 more times by rounding numbers and adding extra value and try to run IK again
    // Other optimisation techniques can be used, 2 different methods are shown for demonstration purposes
    else 
    {   
        //round the numbers to 1 decimal place
        int x1 = floor(msg.data[0]*10+0.5);
        int z1 = floor(msg.data[2]*10+0.5);
        Eigen::Vector3d pos2(x1/10.0, msg.data[1], z1/10.0);
        bool found_ik2 = calcIKSolutionForEffectorPos(pos2, ik_solution);
    
        if(found_ik2) {
            for(int i=0; i<4; ++i) {
                std_msgs::Float64 msg; 
                msg.data = ik_solution[i];
                joint_publishers[i].publish(msg);
            }
        }
        
        else {
        
            //add a random value (i.e. +0.1 to z dimension)
            Eigen::Vector3d pos3(x1/10.0, msg.data[1], z1/10.0+0.1);
            bool found_ik3 = calcIKSolutionForEffectorPos(pos3, ik_solution);
    
            if(found_ik3) {
                for(int i=0; i<4; ++i) {
                    std_msgs::Float64 msg; 
                    msg.data = ik_solution[i];
                    joint_publishers[i].publish(msg);
                }
            }
        }
        
    }*/
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "arm_ik_solver");
    
    ros::NodeHandle n;
    
    joint_publishers.push_back(
        n.advertise<std_msgs::Float64>("shoulder_pitch_controller/intermediate_command", 5)
    );
    joint_publishers.push_back(
        n.advertise<std_msgs::Float64>("elbow_flex_controller/intermediate_command", 5)
    );
    joint_publishers.push_back(
        n.advertise<std_msgs::Float64>("wrist_roll_controller/intermediate_command", 5)
    );
    joint_publishers.push_back(
        n.advertise<std_msgs::Float64>("claw_controller/intermediate_command", 5)
    );
    
    ros::Subscriber pos_sub = n.subscribe("pos_for_IK", 5, gotPosition);
    
    ros::spin();
    
    return 0; 
}
