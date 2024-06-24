# Dobot controller with RViz

This repository contains the final project for the robotics course. It integrates the Dobot controller with RViz
## Table of Contents

- [Installation](#Installation)
- [Usage](#Usage)
- [Reference](#Reference)

## Requirement
- Ubuntu 20.04
- ROS noetic
  If you haven't installed ROS, you can follow the guide below.
  http://wiki.ros.org/noetic/Installation/Ubuntu
## Installation

Instructions on how to install and setup the project.

### Create Workspace
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
<<The name of the workspace that I recommend is "dobot">>

### Instalation package
```sh
cd ~/catkin_ws/src
git clone https://github.com/FarrelCulbert/dobot_ros_rviz.git
cd ../
catkin_make
```
## Usage
### Launch roscore
```sh
roscore
```
### Launch Controller for Dobot
You need to connect the Dobot to a laptop or computer.
```sh
rosrun dobot_driver controller
```
There are some cases where the port permissions need to be configured, such as using `sudo chmod 666 /dev/ttyUSB0` or `sudo chmod +x /dev/ttyUSB0`. Replace `ttyUSB` with the connected USB port.
### Launch Teleop Twist Keyboard
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
### Launch Controller for Rviz
```sh
rosrun dobot controller.py
```
### Launch Rviz
```sh
roslaunch dobot display_dobot.launch
```
## Reference
### Dobot URDF Model 
https://github.com/Alreschas/dobot_magician_urdf
### Dobot Ros for Control
https://github.com/paddenstoeltje/dobot_ros

