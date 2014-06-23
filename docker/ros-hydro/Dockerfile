# Based on the Docker file found at:
#
#	https://github.com/opencog/ros/blob/master/hydro/Dockerfile
#
# Original maintainer Mandeep Singh Bhatia "mandeep.singh.bhatia@gmail.com"
# Original maintainer David Hart "dhart@opencog.org"

FROM ubuntu:12.04

MAINTAINER Rich Wareham <rjw57@cam.ac.uk>

# Use the "noninteractive" debconf frontend since we're installing in a non-interactive way.
ENV DEBIAN_FRONTEND noninteractive

# Make sure that all package sources are up-to-date and that we have the latest
# versions of system packages.
RUN apt-get -y update
RUN apt-get -y upgrade

# Install package containing the CLI tools we'll be using. N.B. after "precise"
# the add-apt-repository command moved to software-properties-common.
RUN apt-get -y install apt-utils
RUN apt-get -y install wget python-software-properties sudo

# Below we're using https so make sure the ca-certificates are installed
RUN apt-get -y install ca-certificates

# Install the ROS package signing key and update the source.list
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
RUN add-apt-repository -y "deb http://packages.ros.org/ros/ubuntu precise main"

# Fetch ROS package repo details
RUN apt-get -y update

# Install ROS base
RUN apt-get -y install ros-hydro-ros-base

# Install additional ROS packages
RUN apt-get -y install python-rosinstall

# Initialise rosdep
RUN rosdep init

# Create ros user as a user with serial port permissions
RUN adduser --gecos "ROS User" --disabled-password ros
RUN usermod -a -G dialout ros

# As ros:
USER ros

# HOME needs to be set explicitly. Without it, the HOME environment variable is
# set to "/"
RUN echo "source /opt/ros/hydro/setup.bash" >> /home/ros/.bashrc
RUN HOME=/home/ros rosdep update

# The default command is to launch a login shell as ros
WORKDIR /home/ros
CMD HOME=/home/ros /bin/bash -l
