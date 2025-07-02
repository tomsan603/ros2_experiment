#!/bin/bash

#export ROBOT_IP=${ROBOT_IP:-192.168.1.20}

CONTAINER=ros2_experiment

# Start the ROS2 container
#docker compose build --no-cache
docker compose up -d

xhost +local:docker

docker exec -it $CONTAINER bash