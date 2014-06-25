FROM sigproc/ros:hydro

MAINTAINER Rich Wareham <rjw57@cam.ac.uk>

# As root...
USER root

# Install additional packages
RUN apt-get -y install ros-hydro-moveit-full
RUN apt-get -y install ros-hydro-dynamixel-motor ros-hydro-dynamixel-controllers
RUN apt-get -y install ros-hydro-ivcon ros-hydro-convex-decomposition
RUN apt-get -y install gazebo ros-hydro-gazebo-ros

# Packages for running a virtual GUI in the container
RUN apt-get -y install x11vnc xvfb
RUN apt-get -y install lubuntu-core
RUN apt-get -y install lxterminal
RUN apt-get -y install openssh-server

# Set password for ros and add to sudoers
RUN echo 'ros:ros' | chpasswd
RUN usermod -a -G sudo ros

# Add ssh's required directory
RUN mkdir /var/run/sshd

# Add this repository to the ros user's workspace.
ADD . /home/ros/workspace/src/robotic_surgery
RUN chown -R ros:ros /home/ros/workspace/src/robotic_surgery
