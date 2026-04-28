# humbleble_ws
実機やシミュレーションを動かすときに必要なパッケージたちです。
シミュレーションは実装途中なので、おいおいやっていきます。
またmoveit2を動かすときに、urdfが読み込めなくて実行できないときがあるかもしれません。

## インストール！！

1. リポジトリをクローン
```bash
mkdir -p ~/ros2_humble_ws/src
cd ~/ros2_humble_ws/src

git clone https://github.com/danbo-rusenki/humbleble_ws.git
git clone https://github.com/Ar-Ray-code/YOLOv5-ROS.git

git clone https://github.com/vstoneofficial/mecanumrover3_ros2.git --recurse-submodules
git clone https://github.com/vstoneofficial/vs_rover_options_description.git -b humble

rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

```
ビルドする前に、研究室のAMIR/ros2_basic　をコピーする。
ros2_humble_ws　とは別に先にビルドしておく。

2. ワークスペースをビルド
```bash
cd ~/ros2_humble_ws
colcon build --symlink-install
```




# 実機を動かすコマンド(旧AMIR)
ターミナル1~5はAMIRに搭載しているPCに入ってから実行する。IPアドレスは変更されることがよくあるので、適宜対応する。
```
ssh rover@192.168.11.12 -X
```

## ターミナル1
```
cd amir_basic_ws
source ~/amir_basic_ws/install/local_setup.bash
sudo chmod 666 /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```
## ターミナル2
```
cd uros_ws
source ~/uros_ws/install/local_setup.bash
sudo chmod 666 /dev/ttyUSB1
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -v4
```
## ターミナル3
```
cd humble_ws/
ros2 run domain_bridge domain_bridge bridge_config.yaml
```
## ターミナル4
```
ros2 launch mecanumrover3_bringup robot.launch.py
```
## ターミナル5
```
source ~/humble_ws/install/local_setup.bash
ros2 launch my_utility odom_tf2_broadcaster.launch.py
```
## ターミナル6
```
sudo chmod 777 /dev/ttyUSB2
source ~/humble_ws/install/local_setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py 
```
## ターミナル7
```
source ~/humble_ws/install/local_setup.bash
ros2 launch mecanum_navigation2 bringup_launch.py 
```
## ターミナル8
```
source ~/humble_ws/install/local_setup.bash
ros2 launch amir_driver amir_bringup.launch.py
```

## ターミナル
```
source ~/humble_ws/install/local_setup.bash
ros2 run amir_operation camera_recogi
```
## ターミナル
```
source ~/humble_ws/install/local_setup.bash
ros2 launch yolov5_ros yolov5s_simple.launch.py
```
## ターミナル
```
source ~/camera_ws/install/setup.bash
ros2 launch realsense2_camera rs_launchZ.py
```


# 自分のPCで

## 初期位置移動
```
自分のworkspace を source ~/ ...
ros2 run amir_operation initial_posi
```

