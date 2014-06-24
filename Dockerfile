FROM sigproc/ros:hydro

MAINTAINER Rich Wareham <rjw57@cam.ac.uk>

# As root...
USER root

# Install additional packages
RUN apt-get -y install ros-hydro-moveit-full
RUN apt-get -y install ros-hydro-dynamixel-motor ros-hydro-dynamixel-controllers
RUN apt-get -y install ros-hydro-ivcon ros-hydro-convex-decomposition
RUN apt-get -y install gazebo ros-hydro-gazebo-ros

# Add this repository to the ros user's workspace.
ADD . /home/ros/workspace/src/robotic_surgery
RUN chown -R ros:ros /home/ros/workspace/src/robotic_surgery
