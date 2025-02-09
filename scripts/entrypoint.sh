#!/bin/bash

source /opt/ros/jazzy/setup.bash


BUILD_DIR="/root/ros2_sample/meitec_ros2_sample/build"

#if [ ! -d "$BUILD_DIR" ]; then
#  cd /root/ros2_sample/meitec_ros2_sample
#  colcon build
#fi

cp /root/ros2_sample/scripts/.vimrc ${HOME}/

exec tmux
