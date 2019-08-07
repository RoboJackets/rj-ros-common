FROM ros:melodic
MAINTAINER Oswin So oswinso@gmail.com

# Setup apt to be happy with no console input
ENV DEBIAN_FRONTEND noninteractive

# setup apt tools and other goodies we want
RUN apt-get update --fix-missing && apt-get -y install apt-utils git software-properties-common ssh python-pip && apt-get clean

# Initialize catkin workspace
RUN mkdir -p ~/catkin_ws
WORKDIR ~/catkin_ws
RUN mkdir -p src

COPY . ./src/rj-ros-common

# Install all ROS dependencies that can automatically be installed
RUN /bin/bash -c "rosdep install -iy --from-paths ./src"
