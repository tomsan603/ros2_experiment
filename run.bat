@echo off
set CONTAINER=ros2_experiment

docker compose up -d

docker exec -it %CONTAINER% bash