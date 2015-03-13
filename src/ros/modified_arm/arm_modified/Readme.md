When setting up a new robot model, remember to:
(1) change ($find setup_config) ==> ($find XXpkg_nameXX) in planning_context.launch file
(2) change the XXX.xacro file in the launch file (value of arg)

Note that, when copying from kinematics_smart:
(1) The 'type' for nodes in launch files point to the name of the executable defined in CMakeLists.txt
(2) One row has to be removed from the vector for the static version because there are only 5 joints.
