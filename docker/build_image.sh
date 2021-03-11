#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
IMAGE_REPO="roborts_ros"
ARCH_TYPE="$(arch)"
ROS_VERSION="noetic"
if [ $ARCH_TYPE == "x86_64" -o ${ARCH_TYPE} == "aarch64" ]
then
    IMAGE_TAG="roborts_base_${ARCH_TYPE}"
    if [ $ARCH_TYPE == "x86_64" ]
    then
        BASE_IMAGE="ros:${ROS_VERSION}-ros-base-focal"
    else
        BASE_IMAGE="arm64v8/ros:${ROS_VERSION}-ros-base-focal"
    fi
else
    echo "ARCH_TYPE $ARCH_TYPE is supported. Valid architecture is [aarch64, x86_64]"
    exit 1
fi

cd ${DIR}

sudo docker build \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    -t ${IMAGE_REPO}:${IMAGE_TAG} \
    -f Dockerfile .
