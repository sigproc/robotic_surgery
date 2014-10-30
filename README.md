k2_client
=========
Client application for Kinect for Windows v2

This software is a part of software package for integrating kinect for windows v2 with ROS. The software package is divided into two parts. One part runs on windows machine and dumps the data over the network, while, the other part runs on linux machine which reads the stream and publishes appropriate ROS topics.

This software is meant to be run on the linux side of the system. The corresponding package to run on the windows side is called k2_server and can be found at: https://github.com/personalrobotics/k2_server/releases

Setting up the software
=======================

A) Download the k2_client package from following link.

      https://github.com/personalrobotics/k2_client/releases

B) Before you can use the k2_client package, you need to install libjsoncpp-dev. Use the following code snippet for it.

`sudo apt-get update`

`sudo apt-get install libjsoncpp-dev`

C) Move the k2_client package to your ros workspace **or** add its location the ROS_PACAKGE_PATH environment variable by adding the following line at the end of ~/.bashrc

`export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/TO/PACKAGE/THE/PACKAGE`

D) Edit the value of the parameter "serverNameOrIP" with the IP address or name of your windows machine which runs the server software. Once done, use "rosmake" to build the package.

E) Start all the ROS nodes by running the following command. Make sure that the roscore is running and the environment variable $ROS_MASTER_URI points to it.

`roslaunch k2_client kinect2Client.launch`
