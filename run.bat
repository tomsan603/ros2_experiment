@echo off
set CONTAINER=ros2_experiment

:: Start the ROS2 container
docker compose up -d

:: Windowsではxhost +の代わりにDISPLAY環境変数を確認（docker-compose.ymlで設定済み）
:: set DISPLAY=:0 （必要に応じて）

:: コンテナのbashに入る
docker exec -it %CONTAINER% bash