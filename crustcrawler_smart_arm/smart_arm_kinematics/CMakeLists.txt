cmake_minimum_required(VERSION 2.8.3)
project(smart_arm_kinematics)
find_package(catkin REQUIRED
    COMPONENTS roscpp geometry_msgs tf urdf message_generation
)

add_service_files(DIRECTORY srv FILES SmartArmIK.srv)
generate_messages(DEPENDENCIES geometry_msgs)

add_executable(smart_arm_ik src/smart_arm_ik.cpp)
target_link_libraries(smart_arm_ik ${roscpp_LIBRARIES} ${tf_LIBRARIES} ${urdf_LIBRARIES})

add_executable(smart_arm_ik_service src/smart_arm_node.cpp)
target_link_libraries(smart_arm_ik_service ${roscpp_LIBRARIES} ${tf_LIBRARIES} ${urdf_LIBRARIES})
add_dependencies(smart_arm_ik_service ${PROJECT_NAME}_generate_messages_cpp)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
