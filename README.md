# igvc-2023-software
Intelligent Ground Vehicle Competition (Spring 2023) software for the Paradigm Engineering Student Group


## Install

This repository contains the software for the 2023 IGVC (Intelligent Ground Vehicle Competition) developed by Paradigm Engineering.

## Prerequisites

- ROS 2 Foxy Fitzroy installed. Follow the installation guide [here](https://index.ros.org/doc/ros2/Installation/Foxy/).
- Make sure you have `git` installed on your system. If not, you can install it using `sudo apt-get install git`.

## Installation

Follow these steps to set up the IGVC 2023 software on your machine:

1. Clone the repository:

 ```bash
 git clone --recurse-submodules git@github.com:ParadigmEngineering/igvc-2023-software.git src/igvc-2023-software
  ```

2. Installing PCL ROS Package

Before building the project, make sure you have the `ros-$ROS_DISTRO-pcl-ros` package installed. Replace `$ROS_DISTRO` with your ROS distribution (e.g., melodic, noetic, foxy, etc.). 

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-pcl-ros
sudo apt-get install ros-$ROS_DISTRO-pcl-conversions
```

3. Install

```bash
cd ~/igvc-2023-software
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

