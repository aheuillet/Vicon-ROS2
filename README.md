# ROS2Vicon

[![Build Status](https://travis-ci.com/aheuillet/Vicon-ROS2.svg?branch=master)](https://travis-ci.com/aheuillet/Vicon-ROS2)

> This C++ program retrieves data from a compatible Vicon software (Nexus, Shogun, Tracker, Blade) and publish it on a ROS2 topic.

## Requirements

Recommanded OS : Ubuntu 18.04 LTS (Bionic Beaver)

If you are using the recommanded OS precised above, you can simply run the `install_ubuntu_bionic.sh` to install all dependencies.
Otherwise, follow these instructions :

First, make sure that you have installed the latest version of ROS2 (Dashing Diademata).
If not, follow the instructions at [the ROS2 website](https://index.ros.org/doc/ros2/Installation/Dashing/).

Then, install [Colcon](https://colcon.readthedocs.io/en/released/index.html) and [CMake](https://cmake.org/) :

`sudo apt install python3-colcon-common-extensions && sudo apt install cmake`

You will also need to install wxWidgets in order to build the GUI :

`sudo apt install wx-common wx3.0-headers libwxgtk3.0-dev libwxgtk-media3.0-dev libwxgtk3.0-gtk3-dev libcanberra-gtk-module libcanberra-gtk3-module`

And :

`sudo ln -sv /usr/include/wx-3.0/wx /usr/include/wx`

## Quickstart

:warning: Do not forget to source the global ROS2 workspace : `source /opt/ros/dashing/setup.bash`

First, run the following command to install the package's shared libraries:

`sudo sh install_libs.sh`

Then, run the following command to build the executable:

`cd src/nexus_interface/ && colcon build`

Once it is finished, you have to source the current workspace so that ROS2 will be able to find the executable :

`source src/nexus_interface/install/setup.bash`

Finally, you can run the program using the following command :

`ros2 run nexus_package client`
