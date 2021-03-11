# RoboRTS-Base

This repo provides latest ROS driver for RoboMaster AI Robot 2019/2020.

## 1. Install dependencies

Before running AI Robot ROS driver, ROS and some dependencies must be installed.

### 1.1 ROS installation
 
For ROS installation, please refer to [ROS Installation Guide](https://www.ros.org/install/)

### 1.2 dependencies installation

After ROS installation, run the following command to install some ROS packages and google-glog library: 

```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
     ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-geometry-msgs libgoogle-glog-dev
```

## 2. Build

1. Get RoboRTS-Base from GitHub :
```bash
git clone https://github.com/RoboMaster/RoboRTS-Base.git catkin_ws/src
```

2. Use the following command to build the package

```bash
cd catkin_ws
catkin_make
```

## 3. Configuration
1. Create the `udev` rule file:

```bash
./src/scripts/udev.sh create
```

2. Verify the `udev` rule file:

```bash
./src/scripts/udev.sh check
```

3. Connect the AI Robot through USB cable from `Chassis Development Board Type C`, then check the status of device connection:

```bash
./src/scripts/udev.sh status
```

## 4. Run

Run the following command to run the driver node from launch file:
```bash
source devel/setup.bash
roslaunch roborts_base base.launch
```

## 5. Run in the Docker Conatiner

Refer to [Docker Conatiner Document](docs/docker.md)

## 6. Node API

Refer to [Node API Document](docs/node_api.md)

## 7. Related Repo

1. Perception/Navigation/Decision Stack Repo: [RoboRTS](https://github.com/RoboMaster/RoboRTS)

2. Embedded STM32 Code Repo: [RoboRTS-Firmware](https://github.com/RoboMaster/RoboRTS-Firmware)

## 8. Support

You can get support from RoboMaster with the following methods :

- Send email to [robomaster@dji.com](mailto:robomaster@dji.com) with a clear description of your problem and your setup
- Report issue on github

## 9. Copyright and License
RoboRTS-Base is provided under the [GPL-v3](LICENSE).