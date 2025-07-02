# ベースイメージ: ROS 2 Humble
FROM ros:humble

# 環境変数: インタラクティブでないインストール用
ENV DEBIAN_FRONTEND=noninteractive

# 基本開発ツール
RUN apt-get update -q && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    vim \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Gazebo（シミュレーション）とRViz2（可視化）を完全インストール
RUN apt-get update -q && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-ros-gz \
    ros-humble-rviz2 \
    ros-humble-ign-ros2-control \
    ros-humble-ros2-control\
    ros-humble-ros2-controllers \

    libasio-dev \
    libcurlpp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update -q
RUN sudo apt-get install -y libgz-gui7-dev libgz-rendering7-dev libgz-sensors7-dev
# カメラセンサ（RealSense）
RUN apt-get update -q && apt-get install -y --no-install-recommends \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# 深層学習（PyTorch＋依存ライブラリ）
RUN pip3 install --no-cache-dir \
    torch torchvision \
    numpy \ 
    opencv-python

# TensorRT（GPU高速化、ホストにNVIDIAドライバ必要）
# RUN apt-get update -q && apt-get install -y --no-install-recommends \
#     libnvinfer-dev \                  # TensorRT開発ライブラリ
#     libnvinfer-plugin-dev \           # TensorRTプラグイン
#     && rm -rf /var/lib/apt/lists/*

# URロボットアームとMoveIt2を完全インストール
RUN apt-get update -q && apt-get install -y --no-install-recommends \
    # ros-humble-ur \                  
    ros-humble-ur-moveit-config \ 
    ros-humble-moveit \ 
    && rm -rf /var/lib/apt/lists/*

# ワークスペース作成
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

#sudo apt update
#sudo apt install ros-humble-ros-gz-sim
#sudo apt update
#sudo apt install ros-humble-ros-gz \
                # ros-humble-ros-gz-bridge \
                 #ros-humble-ros-gz-sim \
                 #ros-humble-ros-gz-image \
                 #ros-humble-ros-gz-interfaces \
                 #ros-humble-ros-gz-sim-demos \
                 #ros-humble-gz-ros2-control
# UR関連リポジトリをクローン
RUN cd /ros2_ws/src && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git -b humble --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git -b humble --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git -b humble --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git --depth 1 && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git -b humble --depth 1 && \
    git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git && \
    git clone https://github.com/ian-chuang/serial-ros2.git -b humble&& \
    git clone https://github.com/gbartyzel/ros2_net_ft_driver.git -b humble


# 依存関係をすべて解決
RUN apt-get update -q && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# ワークスペースビルド
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 環境設定をbashrcに
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# エントリーポイントとコマンド
ENTRYPOINT []
CMD ["/bin/bash"]