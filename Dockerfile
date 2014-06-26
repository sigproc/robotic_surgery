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
RUN apt-get -y install xserver-xephyr

# Packages for OpenRave. (See http://moveit.ros.org/wiki/Kinematics/IKFast)
RUN add-apt-repository ppa:openrave/release
RUN apt-get -y update
RUN apt-get -y install openrave0.8-dp-ikfast
RUN apt-get -y install openrave0.8-dp-plugins-base
RUN apt-get -y install openrave0.8-dp-plugin-oderave openrave0.8-dp-plugin-textserver

RUN apt-get -y install locales

# Set password for ros and add to sudoers
RUN echo 'ros:ros' | chpasswd
RUN usermod -a -G sudo ros

# Add ssh's required directory
RUN mkdir /var/run/sshd

# Add this repository to the ros user's workspace.
ADD . /home/ros/workspace/src/robotic_surgery
RUN chown -R ros:ros /home/ros/workspace/src/robotic_surgery

# Copy ssh config
ADD config/ssh /home/ros/.ssh
RUN chown -R ros:ros /home/ros/.ssh
RUN chmod -R og-rwx /home/ros/.ssh

# Default command is to run a ssh daemon in the container
CMD ["/usr/sbin/sshd", "-D"]
