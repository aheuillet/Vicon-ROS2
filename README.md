# ROS2Vicon

> This C++ program retrieves data from a compatible Vicon software (Nexus, Shogun, Tracker, Blade) and publish it on a ROS2 topic.

## Requirements

Recommanded OS : Ubuntu 18.04 LTS (Bionic Beaver)

First, make sure that you have installed the latest version of ROS2 (Dashing Diademata).
If not, follow the instructions at [the ROS2 website](https://index.ros.org/doc/ros2/Installation/Dashing/).

Then, install [Colcon](https://colcon.readthedocs.io/en/released/index.html) and [CMake](https://cmake.org/) :

`sudo apt install python3-colcon-common-extensions && sudo apt install cmake`

## Quickstart

:warning: Do not forget to source the global ROS2 workspace : `source /opt/ros/dashing/setup.bash`

First, run the following command to install the package's dependencies and build the executable :

`sudo sh install.sh`

Once it is finished, you have to source the current workspace so that ROS2 will be able to find the executable :

`source src/nexus_interface/install/setup.bash`

Finally, you can run the program using the following command :

`ros2 run nexus_package client`
