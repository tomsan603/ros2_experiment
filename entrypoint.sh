ros2 launch ur_robotiq_description view_ur.launch.py
ros2 launch ur_robotiq_description view_ur_gripper.launch.py
ros2 launch ur_robotiq_description spawn_ur3e_camera.launch.py
ros2 launch ur_robotiq_description spawn_ur3e_camera_gripper.launch.py
ros2 launch ur_robotiq_description gz_sim.launch.py
docker exec -it ros2_experiment bash
apt update&&apt install ros-humble-ign-ros2-control
apt update&&apt install ros-humble-ros-gz