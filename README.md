# humbleble_ws
実機やシミュレーションを動かすときに必要なパッケージたちです。

- sim-ign
- light-sim-ign
- jikki_test


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

## カメラ映像を自分のpcに表示させる
```
自分のworkspace を source ~/ ...
ros2 run amir_navigation image_show 
```
