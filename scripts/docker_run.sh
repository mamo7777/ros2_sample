#!/bin/bash

CONTAINER_NAME="meitec_ros2_container"

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo "Attaching to existing container: ${CONTAINER_NAME}"
  docker exec -it ${CONTAINER_NAME} bash
else
  echo "Starting new container: ${CONTAINER_NAME}"
  docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${HOME}/ros2_sample:/root/ros2_sample \
    --name ${CONTAINER_NAME} \
    meitec_ros2_docker
fi
