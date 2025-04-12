#!/bin/bash

CONTAINER=ros2_experiment

# Start the ROS2 container
docker compose up -d

# xhost +

docker exec -it $CONTAINER bash