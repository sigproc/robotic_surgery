FROM sigproc/ros:hydro

MAINTAINER Rich Wareham <rjw57@cam.ac.uk>

# Add this repository to the ros user's home.
USER root
ADD . /home/ros/robotic-surgery
RUN chown -R ros:ros /home/ros/robotic-surgery
