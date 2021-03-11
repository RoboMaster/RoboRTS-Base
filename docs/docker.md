# Run RoboRTS-Base with Docker
## 1. Docker Installation
Get the installation script and install the docker:

```bash
curl -fsSL https://get.docker.com | bash -s docker
```

## 2. Build Docker Image
Get RoboRTS-Base from GitHub :

```bash
git clone https://github.com/RoboMaster/RoboRTS-Base.git catkin_ws/src
```
Build the docker image:

```bash
cd catkin_ws/src/docker
./build_image.sh
```
The default image is based on ROS Noetic and compatible with x86 and arm64 architecture.
If you want to modify the image configuration, please refer to [build_image.sh](../docker/build_image.sh)

## 3. Config and Build the Package in the Docker Container

Config the udev rules and build the package in the docker container:

```bash
./config_and_make.sh
```

## 4. Run the Package in the Docker Container

Run the ROS driver from launch file in the docker container:

```bash
./launch.sh
```