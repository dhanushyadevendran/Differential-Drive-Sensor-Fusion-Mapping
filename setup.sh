#!/bin/bash
set -e

echo "Installing ROS2 Packages..."
sudo apt update && sudo apt upgrade -y

sudo apt install -y ros-foxy-desktop
source /opt/ros/foxy/setup.bash

sudo apt install -y gazebo11 ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-plugins
sudo apt install -y ros-foxy-robot-state-publisher \
                    ros-foxy-joint-state-publisher-gui \
                    ros-foxy-urdf \
                    ros-foxy-xacro \
                    ros-foxy-tf2-tools \
                    ros-foxy-diff-drive-controller \
                    ros-foxy-gazebo-ros2-control

sudo apt install -y python3-colcon-common-extensions \
                    python3-rosdep \
                    python3-argcomplete \
                    python3-numpy python3-matplotlib python3-pandas git

sudo rosdep init || echo "rosdep already initialized"
rosdep update