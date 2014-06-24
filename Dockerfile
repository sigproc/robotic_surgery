FROM sigproc/robotic-surgery:ros-hydro

MAINTAINER Rich Wareham <rjw57@cam.ac.uk>

# Add this repository to the ros user's home. NOTE: this assumes that this Dockerfile is
# located at <something>/<something>/Dockerfile
USER root
ADD . /home/ros/robotic-surgery
RUN chown -R ros:ros /home/ros/robotic-surgery

# The default command is to run a login shell as the ros user
USER ros
WORKDIR /home/ros
CMD HOME=/home/ros /bin/bash -l
