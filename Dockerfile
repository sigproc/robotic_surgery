FROM sigproc/ros:hydro

MAINTAINER Rich Wareham <rjw57@cam.ac.uk>

# Add this repository to the ros user's workspace.
USER root
ADD . /home/ros/workspace/src/robotic_surgery
RUN chown -R ros:ros /home/ros/workspace/src/robotic_surgery

# Build the repository
USER ros
WORKDIR /home/ros/workspace/
RUN HOME=/home/ros /bin/bash -c 'source ~/workspace/devel/setup.bash; catkin_make'
