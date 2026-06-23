# humbleble_ws
実機やシミュレーションを動かすときに必要なパッケージたちです。
シミュレーションは実装途中なので、おいおいやっていきます。
またmoveit2を動かすときに、urdfが読み込めなくて実行できないときがあるかもしれません。

## インストール！！

1. リポジトリをクローン
```bash
mkdir -p ~/ros2_humble_ws/src
cd ~/ros2_humble_ws/src

git clone https://github.com/danbo-rusenki/humbleble_ws.git -b jikki_test

rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

```

2. ワークスペースをビルド
```bash

sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-velocity-controllers \
  ros-humble-effort-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-gz-ros2-control

cd ~/ros2_humble_ws
colcon build --symlink-install
```
3. pythonファイルに実行権限を付与、ファイルのあるディレクトリに移動するか、ファイルの場所を指定してください。  ここやらなくてもいいかも〜;o;
```bash
chmod +x rover_twist_relay.py

chmod +x joint_state_filter.py
```

## コマンド

```bash
cd amir_basic_ws
source ~/amir_basic_ws/install/local_setup.bash
sudo chmod 666 /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```
```bash
cd uros_ws
source ~/uros_ws/install/local_setup.bash
sudo chmod 666 /dev/ttyUSB1
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -v4
```
```bash
source install/setup.bash 
ros2 launch amir_driver amir_bringup.launch.py
```
```bash
source install/setup.bash 
ros2 launch mecanumrover3_bringup robot.launch.py
```
```bash

ros2 run domain_bridge domain_bridge bridge_config.yaml 
```

1. gazebo 立ち上げ
```bash
source install/setup.bash 
ros2 launch amir_gazebo gazebo_bringup.launch.py
```
倉庫
```bash
source install/setup.bash 
ros2 launch amir_gazebo gazebo_bringup2.launch.py
```
壁
```bash
source install/setup.bash 
ros2 launch amir_gazebo gazebo_bringup3.launch.py
```
```bash
source install/setup.bash 
ros2 launch mecanumrover3_gazebo spawn_koteibox.launch.py 
```


　箱を出現させる
10個出す
```bash  
source install/setup.bash 
ros2 launch mecanumrover3_gazebo spawn_10box.launch.py
```
```bash
source install/setup.bash 
ros2 launch mecanumrover3_gazebo spawn_multibox.launch.py 
```
　迷路を出現させる
```bash
source install/setup.bash 
ros2 launch mecanumrover3_gazebo spawn_wor.launch.py scale:=0.001
```
2. 初期位置移動
```bash
source install/setup.bash 
ros2 run amir_operation initial_posi_gz
```

3. moveit2 
```bash
source install/setup.bash 
ros2 launch amir_moveit_config moveit_gazebo.launch.py 
```

4. moveit2に指示送る
```bash
source install/setup.bash 
ros2 launch amir_operation pick_place_fix_launch.py 
```

10個ピック、障害物避けつつ
```bash 
source install/setup.bash 
ros2 launch amir_operation pick_place_10_launch.py 
```

5. nav2 
```bash
source install/setup.bash 
ros2 launch mecanum_navigation2 bringup_launch.py
```
6. action server 
```bash
source install/setup.bash 
ros2 launch amir_operation pick_and_place_launch.py
```

7. bt send 
```bash
source install/setup.bash 
ros2 launch bt_generator bt_send_xml_launch.py
```

8. bt exe
```bash
source install/setup.bash
ros2 launch ros2_behavior_tree bt_executor_launch.py
```

9. 座標確認
```bash
source install/setup.bash
ros2 run my_utility gz_pose_filter 
```

## VR/MR テレオペ (MoveIt Servo で手先 Twist 操作)

手先の Twist 指令を MoveIt Servo (微分IK) で関節角ストリームに変換し、
`forward_position_controller` 経由で Gazebo のアームを動かす。
`arm_controller` (JTC) と `forward_position_controller` は同じ position command
interface を奪い合うため、同時に active にできない。switch_controllers で切り替える。

データ流路:
手先Twist → `/servo_node/delta_twist_cmds` → moveit_servo → `/forward_position_controller/commands` → Gazebo

1. gazebo 立ち上げ (上記「コマンド」1 と同じ)
```bash
source install/setup.bash
ros2 launch amir_gazebo gazebo_bringup.launch.py
```

2. Servo 一式起動 (servo_node + joint_state_filter + forward_position_controller を --inactive で spawn)
```bash
source install/setup.bash
ros2 launch amir_operation vr_servo_launch.py
```

3. 出力先を Servo へ切替 (JTC を止めて forward_position_controller を有効化)
```bash
source install/setup.bash
ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller
```

4. 手先 Twist を流す (いずれか1つ。Servo は各ノードが自動で start する)

   a. 動作確認用 (正弦波 Twist を自動で publish)
   ```bash
   source install/setup.bash
   ros2 run amir_operation servo_test_twist
   ```

   b. キーボード操作
   ```bash
   source install/setup.bash
   ros2 run amir_operation servo_keyboard_twist
   ```
   速度保持方式: 押した方向に動き続け、`space` で停止する (ターミナルのキーリピートに依存しない確実な方式)。
   キー割り当て (base_footprint 基準):
   - `w`/`s` : +X/-X (前後)
   - `a`/`d` : +Y/-Y (左右)
   - `r`/`f` : +Z/-Z (上下)
   - `space` : 停止 (全軸ゼロ) , `[`/`]` : 速度ダウン/アップ , `q` : 終了
   - 各軸は独立保持。w の後 a で斜め移動。単一方向に戻すには space で一度止めてから押し直す。

   速度や基準フレームはパラメータで変更可:
   ```bash
   ros2 run amir_operation servo_keyboard_twist --ros-args -p linear_speed:=0.05 -p z_speed:=0.05 -p command_frame:=tcp_link
   ```

   c. VR ブリッジ (`/vr/controller_pose` PoseStamped + `/vr/enable` Bool を別途 publish)
   ```bash
   source install/setup.bash
   ros2 run amir_operation vr_twist_bridge
   ```

   d. キーボード操作 (3DOF位置ジョグ版 / 5軸で並進だけ素直に動かす)【推奨】
   ```bash
   source install/setup.bash
   ros2 run amir_operation servo_keyboard_jog      # 引数不要
   ```
   ※b版(servo_keyboard_twist)は Servo の Cartesian経路(6x5ヤコビアン)を使うため、
     angular=0 が拘束になり5軸では過剰拘束になる。d版は自前で「3x5の位置ヤコビアン」だけで
     微分IK(dq = J_pos^+ v)を解き、関節ジョグ `/servo_node/delta_joint_cmds`(control_msgs/JointJog)
     へ渡す。余る2自由度は冗長として自由になり、並進3方向が素直に動く。
     ヤコビアンは MoveIt RobotState::getJacobian(=Servo本体と同じ planning フレーム)で計算。
     URDF は既定で sim と同じ moveit_config の xacro を自動展開して使う(base取付けの -90° まで
     一致するのでフレーム補正不要)。キー割り当てはb版と同じ(w/s/a/d/r/f, space=停止)。
     別機体なら `-p xacro_path:=... -p srdf_path:=... -p group:=... -p tip_link:=...` で上書き。
     キーボード入力は `ros2 run` で前面実行すること(`ros2 launch` だと stdin が届かない)。

5. 自律動作 (JTC) へ戻す
```bash
source install/setup.bash
ros2 control switch_controllers --deactivate forward_position_controller --activate arm_controller
```

動作確認 (別端末):
```bash
ros2 control list_controllers                          # forward_position_controller が active か
ros2 topic echo /servo_node/status                     # 0=正常 / 3=特異点 / 4=関節限界
ros2 topic echo /forward_position_controller/commands  # 関節角指令が出ているか
```


