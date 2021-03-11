ARG BASE_IMAGE=ros:noetic-ros-base-focal
FROM ${BASE_IMAGE}
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-geometry-msgs libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /root/catkin_ws/src
