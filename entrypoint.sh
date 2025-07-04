ros2 launch ur_robotiq_description view_ur.launch.py
ros2 launch ur_robotiq_description view_ur_gripper.launch.py
ros2 launch ur_robotiq_description spawn_ur3e_camera.launch.py
ros2 launch ur_robotiq_description spawn_ur3e_camera_gripper.launch.py
ros2 launch ur_robotiq_description gz_sim.launch.py
docker exec -it ros2_experiment bash
apt update&&apt install ros-humble-ign-ros2-control
apt update&&apt install ros-humble-ros-gz
ros2 launch ur_robotiq_description gz_sim.launch.py world_file:="/ros2_ws/src/my_packages/ur_robotiq_description/worlds/my_world.sdf"
ros2 launch ur_robotiq_description gz_sim.launch.py world_file:="/ros2_ws/src/my_packages/ur_robotiq_description/worlds/my_world.sdf"
sudo chown -R karuto:karuto ~/ros2_ur
git clone https://github.com/panagelak/rq_fts_ros2_driver.git
ros2 launch my_robot_description view_model.launch.py
ros2 launch my_robot_simulation my_robot_sim.launch.py world_file:="/ros2_ws/src/my_packages/my_robot_simulation/worlds/my_world.sdf"