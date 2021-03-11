#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# get the IMAGE_REPO and IMAGE_TAG
IMAGE_REPO="roborts_ros"
ARCH_TYPE="$(arch)"
if [ ${ARCH_TYPE} == "x86_64" -o ${ARCH_TYPE} == "aarch64" ]
then
    IMAGE_TAG="roborts_base_${ARCH_TYPE}"
else
    >&2 echo "[ERROR] ARCH_TYPE ${ARCH_TYPE} is supported. Valid architecture is [aarch64, x86_64]"
    read -p "[INFO] Build Process Failed" var
    exit 1
fi

# check if the image exists, if not, build the expected image
while [[ "$(sudo docker images -q ${IMAGE_REPO}:${IMAGE_TAG} 2> /dev/null)" == "" ]]
do
    echo "[INFO] Image ${IMAGE_REPO}:${IMAGE_TAG} does not exist, start to build the image."
    sh -c "${DIR}/build_image.sh"
done


HOST_BASE_DIR="$( cd "${DIR}/.." >/dev/null 2>&1 && pwd )"
# config the udev rule in the host machine
sh -c "${HOST_BASE_DIR}/scripts/udev.sh create"

# restore tmp building file for the docker workspace
mkdir -p "${DIR}/tmp/build" "${DIR}/tmp/devel"

# create the container
DOCKER_CATKIN_WS_DIR="/root/catkin_ws"
CONTAINER_NAME="roborts_ros"
echo "[INFO] Run the Container ${CONTAINER_NAME} from image ${IMAGE_REPO}:${IMAGE_TAG}"
echo "[INFO] Mapping the ros directory from local path ${HOST_BASE_DIR} to docker path ${DOCKER_CATKIN_WS_DIR}/src "
sudo docker run \
     -it \
     --rm \
     --net=host \
     --name ${CONTAINER_NAME} \
     --privileged \
     --volume /dev:/dev  \
     --volume ${HOST_BASE_DIR}/roborts_base:${DOCKER_CATKIN_WS_DIR}/src/roborts_base \
     --volume ${HOST_BASE_DIR}/roborts_msgs:${DOCKER_CATKIN_WS_DIR}/src/roborts_msgs \
     --volume ${HOST_BASE_DIR}/scripts:${DOCKER_CATKIN_WS_DIR}/src/scripts           \
     --volume ${HOST_BASE_DIR}/docker/tmp/build:${DOCKER_CATKIN_WS_DIR}/build        \
     --volume ${HOST_BASE_DIR}/docker/tmp/devel:${DOCKER_CATKIN_WS_DIR}/devel        \
     ${IMAGE_REPO}:${IMAGE_TAG} \
     /bin/bash ${DOCKER_CATKIN_WS_DIR}/src/scripts/catkin_make.sh
# check docker run result
if [ $? != 0 ]
then
    echo "[WARN] Container already exists. Please wait for stopping and restarting the current container. "
    sudo docker stop ${CONTAINER_NAME} >/dev/null 2>&1
    sudo docker rm ${CONTAINER_NAME} >/dev/null 2>&1
    sudo docker run \
         -it \
         --rm \
         --net=host \
         --name ${CONTAINER_NAME} \
         --privileged \
         --volume /dev:/dev  \
         --volume ${HOST_BASE_DIR}/roborts_base:${DOCKER_CATKIN_WS_DIR}/src/roborts_base \
         --volume ${HOST_BASE_DIR}/roborts_msgs:${DOCKER_CATKIN_WS_DIR}/src/roborts_msgs \
         --volume ${HOST_BASE_DIR}/scripts:${DOCKER_CATKIN_WS_DIR}/src/scripts           \
         --volume ${HOST_BASE_DIR}/docker/tmp/build:${DOCKER_CATKIN_WS_DIR}/build        \
         --volume ${HOST_BASE_DIR}/docker/tmp/devel:${DOCKER_CATKIN_WS_DIR}/devel        \
         ${IMAGE_REPO}:${IMAGE_TAG} \
         /bin/bash ${DOCKER_CATKIN_WS_DIR}/src/scripts/catkin_make.sh
fi

