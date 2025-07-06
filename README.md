# リポジトリの説明
このリポジトリはur3eとrobotiq社の2f_85グリッパー、robotiq社のFT300トルクセンサーを用いて
シミュレーション実験と実機実験をするためのものです  
ROS2の基礎知識さえ習得していれば誰もが自分の好きなような実験ができるようにしています  
注）ROS2 HumbleとIgnition Fortressを使用しています

# １初期準備
DockerとGitをダウンロードしてください
# ２実験環境を起動＆停止する方法
## Linuxの場合

1. 好きなディレクトリで以下のコードを実行してください

    ```bash
    git clone https://github.com/tomsan603/ros2_experiment.git　
    ```

2. 実行権限を付与します(初回のみ)

    ```bash
    cd ./ros2_experiment
    chmod +x ./run.bash
    ```

3. Docker Containerを起動します  
（$→#になったら起動しています）

    ```bash
    ./run.bash
    ```

4. Docker Containerを停止します  
(#→$になったらしている停止しています)

    ```bash
    exit
    ```
5. 実行権限を付与します(初回のみ)

    ```bash
    chmod +x ./stop.bash
    ```

6. Docker Containerを停止します

    ```bash
    ./stop.bash
    ```

## Windowsの場合

1. 好きなディレクトリで以下のコードを実行してください

    ```bash
    git clone https://github.com/tomsan603/ros2_experiment.git　
    ```

2. ディレクトリを移動します

    ```bash
    cd ./ros2_experiment
    ```

3. Docker Containerを起動します  
（$→#になったら起動しています）

    ```bash
    ./run.bat
    ```

4. Docker Containerを停止します  
(#→$になったらしている停止しています)

    ```bash
    exit
    ```

5. Docker Containerを停止します

    ```bash
    ./stop.bat
    ```

# ３実験方法
## シミュレーション実験の場合
1. Gazeboを立ち上げます  
MoveItなしの場合

    ```bash
    ros2 launch my_robot_simulation my_robot_sim.launch.py
    ```
   MoveItありの場合
    ```bash
    ros2 launch my_robot_simulation my_robot_sim_moveit.launch.py
    ```
2. 自作制御ノードを起動してgazebo内のロボットを動かします(xxxは自作した任意の名称)

    ```bash
    ros2 run my_robot_apps xxx.py
    ```
## 実機実験の場合
1. 実機ドライバーを立ち上げます  
MoveItなしの場合

    ```bash
    ros2 launch my_robot_bringup my_robot_control.launch.py
    ```
   MoveItありの場合
    ```bash
    ros2 launch my_robot_bringup my_robot_control_moveit.launch.py
    ```
2. 自作制御ノードを起動して実機のロボットを動かします(xxxは自作した任意の名称)

    ```bash
    ros2 run my_robot_apps xxx.py
    ```
# ４補足（パッケージの説明）
## my_robot_description
ロボットのモデル（形状、関節、センサー）を定義するパッケージです
## my_robot_simulation
Gazeboを起動し、ROS2と接続するためのパッケージです
## my_robot_bringup
実機のロボットを起動し、ROS2と接続するためのパッケージです
## my_robot_apps
自作のアプリケーションロジックを実装するパッケージです　Folkして独自のコードを追加して実行することをお勧めします
## my_robot_moveit_config
ロボットをMoveItをもちいて動かすための設定ファイルを含むパッケージです
## ur_robotiq_description&my_gripper_controller
このリポジトリを作るのにtomsan603が使用した学習用パッケージです。Gazebo Classicで動きます


