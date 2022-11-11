ARG BASE_IMAGE=ros:noetic-ros-base-focal
FROM ${BASE_IMAGE}
ADD sources.list /etc/apt/ #change the source into tuna's mirror
# install ros packages
RUN sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \ #setup sources.list using the tuna's mirror in order to install the following ROS packages and google-glog library without errors
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-geometry-msgs libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /root/catkin_ws/src
