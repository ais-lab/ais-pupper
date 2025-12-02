#!/bin/bash
######################################################################################
# ROS2
#
# This stack will consist of ROS2 install
#
# To install
#    ./pc_install.sh
######################################################################################

# Update package lists
cd ~
sudo apt update

# Install ROS 2 Humble setup scripts
if ! [ -d "ros2_setup_scripts_ubuntu" ]; then
  git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
fi
~/ros2_setup_scripts_ubuntu/ros2-humble-ros-base-main.sh
source /opt/ros/humble/setup.bash

# Create ROS 2 workspace and clone Mini Pupper ROS repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
if ! [ -d "mini_pupper_ros" ]; then
  git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev mini_pupper_ros
fi
vcs import < mini_pupper_ros/.minipupper.repos --recursive

# Install dependencies and build the ROS 2 packages
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy
sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins
sudo apt install -y ros-humble-rqt*

######
sudo apt install ros-humble-vision-msgs # fix vision_msgsConfig.cmake err
###### RUN THIS IF YOU GET vision_msgsConfig.cmake NOT FOUND ERROR
# 1. Remove the old key file first
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# 2. Fetch the key with curl and pipe it through gpg --dearmor
# This converts the key into the binary format apt expects
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
#######################

pip3 install simple_pid
colcon build --symlink-install
