#!/bin/bash

# Install ROS Jazzy packages with -y flag to confirm automatically
sudo apt install -y ros-jazzy-ros2-control
sudo apt install -y ros-jazzy-ros2-controllers
sudo apt install -y ros-jazzy-ros-gz
sudo apt install -y ros-jazzy-gz-ros2-control
sudo apt install -y ros-jazzy-joy*
sudo apt install -y ros-jazzy-joint-state-publisher
sudo apt install -y ros-jazzy-xacro
sudo apt install -y ros-jazzy-*-ros2-control
sudo apt install -y ros-jazzy-joint-state-publisher-gui
sudo apt install -y ros-jazzy-turtlesim
sudo apt install -y ros-jazzy-robot-localization
sudo apt install -y ros-jazzy-joy-teleop
sudo apt install -y ros-jazzy-tf-transformations
sudo apt install -y '~nros-jazzy-rqt*'
sudo apt install -y ros-jazzy-twist-stamper
sudo apt install -y ros-jazzy-twist-mux
sudo apt install -y libserial-dev
sudo apt install -y ros-jazzy-navigation2
sudo apt install -y ros-jazzy-nav2-bringup
sudo apt install -y ros-jazzy-nav2-minimal-tb*

echo "All commands executed successfully."
