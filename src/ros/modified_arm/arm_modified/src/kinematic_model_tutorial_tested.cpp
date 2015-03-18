#include <ros/ros.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>

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

int main(int argc, char **argv)
{
    ros::init (argc, argv, "arm_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Start
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("Arm");

    print_kinematic_state(kinematic_state, joint_model_group);

    
    /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    /* Enforce the joint limits for this state and check again*/
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    
    Eigen::VectorXd joints(5);
    joints << 0, -1.9722, 0, 0, 0;

    //Forward Kinematics
    //kinematic_state->setToRandomPositions(joint_model_group);
    kinematic_state->setJointGroupPositions(joint_model_group, joints);
    Eigen::Affine3d end_effector_state;
    end_effector_state = kinematic_state->getGlobalLinkTransform("effector_tip_link");
    
    print_kinematic_state(kinematic_state, joint_model_group);
    
    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
    
    Eigen::Matrix3d r;
    r << -1, 0, 0,
          0, 0, -1,
          0, 1, 0;
    /*double rotation_matrix[3][3] = {
                                    {0.56, 0.26, 0.564},
                                    {0.76, 0.12, 0.134},
                                    {0.62, 0.77, 0.23}
                                    };
                      
    double translation_vector = {0.1, 0, 0};*/
    
    Eigen::Vector3d v;
    v << 0.1, 0, 0.1;
    
    /*end_effector_state.rotate(r);*/
        
    //this is checked to compile    
    Eigen::Affine3d frame_transform;
    frame_transform = kinematic_state->getFrameTransform("effector_tip_link");
    //The translation is currently according to the frame of the end effector. You need to transform it so that it is based on the model frame. I was thinking of using getFrameTransform.
    
    //Convert Affine to Matrix
    Eigen::Matrix3d frame_transform_Matrix;
    frame_transform_Matrix = frame_transform.linear();
    frame_transform_Matrix = frame_transform.matrix().topLeftCorner<3,3>();
    
    //Vector to transform eef position
    Eigen::Vector3d v_prime;
    v_prime = frame_transform_Matrix.inverse()*v;
    
    end_effector_state.translate(v_prime);
    
    std::cout << "v_prime is "<< v_prime << std::endl;
    
    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
    
    //Inverse Kinematics
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

    if (found_ik)
    {
        print_kinematic_state(kinematic_state, joint_model_group);
    }

    else
    {
        ROS_INFO("Did not find IK solution");
    }

    
    //Get the Jacobian
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
    ROS_INFO_STREAM("Jacobian: " << jacobian);
    
    // END_TUTORIAL
    ros::shutdown();
    return 0;
}
