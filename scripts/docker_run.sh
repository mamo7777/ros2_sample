#!/bin/bash

docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${HOME}/ros2_sample:/root/ros2_sample \
  meitec_ros2_docker 
