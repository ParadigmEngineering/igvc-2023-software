# igvc-2023-software
Intelligent Ground Vehicle Competition (Spring 2023) software for the Paradigm Engineering Student Group


## Install

This repository contains the software for the 2023 IGVC (Intelligent Ground Vehicle Competition) developed by Paradigm Engineering.

## Prerequisites

- ROS 2 Foxy Fitzroy installed. Follow the installation guide [here](https://index.ros.org/doc/ros2/Installation/Foxy/).
- Make sure you have `git` installed on your system. If not, you can install it using `sudo apt-get install git`.

Get Nav2
 ```bash
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src
git clone https://github.com/ros-planning/navigation2.git --branch <ros2-distro>-devel
cd ~/nav2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>
colcon build --symlink-install
 ```

## Installation

Follow these steps to set up the IGVC 2023 software on your machine:

1. Clone the repository:

 ```bash
 git clone --recurse-submodules git@github.com:ParadigmEngineering/igvc-2023-software.git src/igvc-2023-software
  ```

2. Installing PCL ROS Package

Before building the project, make sure you have the `ros-$ROS_DISTRO-pcl-ros` package installed. Replace `$ROS_DISTRO` with our current ROS distribution (e.g., melodic, noetic, foxy, etc.). 

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-pcl-ros
sudo apt-get install ros-$ROS_DISTRO-pcl-conversions
sudo apt install ros-$ROS_DISTRO-spatio-temporal-voxel-layer
sudo apt-get install ros-$ROS_DISTRO-robot-localization
sudo apt-get install libjemalloc-dev

export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libjemalloc.so

source /opt/ros/foxy/setup.bash
source install/setup.bash
```

3. Install & Build

```bash
cd ~/igvc-2023-software
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
