# MoveIt Servo テレオペ機能 移植メモ (sim-ign → light-sim-ign など別ブランチ)

メカナム版(`sim-ign`)で実装した「MoveIt Servo による手先テレオペ(VR/キーボード)」を
別ブランチ(単純タイヤ版 `light-sim-ign` 等)へ移植するための変更箇所まとめ。

## 前提: 車輪方式に依存しない
今回の変更は **アーム(`Joint_1..5` / `tcp_link`)だけ** を対象にしている。
メカナム⇔タイヤの違い(走行コントローラ・車輪ジョイント)とは独立なので、基本そのまま移植可。
移植先 `light-sim-ign` には以下が既に存在するため**新規作成不要**:
- `amir740_ros/amir_moveit_config/scripts/joint_state_filter.py`
- `amir740_ros/amir_moveit_config/config/amir_mecanum3.srdf`
- `amir740_ros/amir_moveit_config/config/amir_mecanum3.urdf.xacro`
- `amir740_ros/amir_gazebo/config/arm_controllers.yaml`

---

## A. 追加するファイル (そのままコピー)

| ファイル | 役割 |
|---|---|
| `amir_operation/config/servo.yaml` | MoveIt Servo 設定(group=arm, frame=base_footprint/tcp_link, 出力→forward_position_controller, 100Hz, joint_topic=/joint_states_filtered) |
| `amir_operation/launch/vr_servo_launch.py` | servo_node + joint_state_filter + forward_position_controller(--inactive spawn) を起動 |
| `amir_operation/launch/servo_keyboard_jog_launch.py` | 3DOFジョグ版の launch(robot_description を渡す) |
| `amir_operation/src/servo_test_twist.cpp` | 動作確認用(正弦波 Twist を自動 publish) |
| `amir_operation/src/vr_twist_bridge.cpp` | VR手姿勢(/vr/controller_pose + /vr/enable)→手先 Twist の汎用ブリッジ |
| `amir_operation/src/servo_keyboard_twist.cpp` | キーボード→Twist(Cartesian経路。6×5で過剰拘束ぎみ) |
| `amir_operation/src/servo_keyboard_jog.cpp` | キーボード→関節ジョグ(3×5 位置ヤコビアン。**推奨**) |

---

## B. 変更するファイル

### 1. `amir740_ros/amir_gazebo/config/arm_controllers.yaml`
`gripper_controller` の前に `forward_position_controller` を追記(Servo の出力先):

```yaml
forward_position_controller:
  ros__parameters:
    joints:
      - Joint_1
      - Joint_2
      - Joint_3
      - Joint_4
      - Joint_5
    interface_name: position
```

### 2. `amir_operation/CMakeLists.txt`
- `find_package` に追加: `std_msgs std_srvs moveit_core srdfdom urdf urdfdom ament_index_cpp`
- 実行ファイル追加: `servo_test_twist` `vr_twist_bridge` `servo_keyboard_twist` `servo_keyboard_jog`
- 依存設定:
  ```cmake
  set(dependencies_servo rclcpp geometry_msgs std_msgs std_srvs tf2 tf2_geometry_msgs)
  ament_target_dependencies(servo_test_twist ${dependencies_servo})
  ament_target_dependencies(vr_twist_bridge ${dependencies_servo})
  ament_target_dependencies(servo_keyboard_twist ${dependencies_servo})
  ament_target_dependencies(servo_keyboard_jog
    rclcpp sensor_msgs control_msgs std_srvs moveit_core srdfdom urdf urdfdom ament_index_cpp)
  target_include_directories(servo_keyboard_jog PUBLIC ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(servo_keyboard_jog Eigen3::Eigen)
  ```
- `install(TARGETS ...)` に 4 実行ファイルを追加
- `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})` を追加(servo.yaml をインストール)

### 3. `amir_operation/package.xml`
`<depend>` 追加:
`std_msgs std_srvs trajectory_msgs tf2 tf2_geometry_msgs moveit_core srdfdom urdf urdfdom ament_index_cpp`
`<exec_depend>` 追加: `moveit_servo position_controllers controller_manager`

### 4. `README.md` (任意)
「VR/MR テレオペ (MoveIt Servo)」節を追記。

---

## C. ビルドに必要な apt パッケージ
```bash
sudo apt install -y ros-humble-moveit-servo ros-humble-moveit-core \
  ros-humble-position-controllers ros-humble-control-msgs ros-humble-srdfdom
```

---

## D. 動かし方(移植後)
```bash
# 1) gazebo
ros2 launch amir_gazebo gazebo_bringup.launch.py
# 2) servo一式
ros2 launch amir_operation vr_servo_launch.py
# 3) 出力先を Servo へ
ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller
# 4) キーボード操作(推奨: 3DOFジョグ版, 引数不要)
ros2 run amir_operation servo_keyboard_jog
#    w/s=±X a/d=±Y r/f=±Z space=停止 [/]=速度 q=終了
# 戻す
ros2 control switch_controllers --deactivate forward_position_controller --activate arm_controller
```

---

## E. 移植時の注意 (車輪方式の違いで効く可能性がある点)
1. **base 取付けの回転**: `servo_keyboard_jog` はそのブランチの `amir_mecanum3.urdf.xacro` を
   起動時に xacro 展開して使う(MoveIt の getJacobian = Servo と同じ planning フレーム)。
   よって**そのブランチの台座向きに自動で追従**し、フレームの定数補正は不要。
   ※ sim-ign では `base_joint` に `rpy=0 0 -1.5708`(-90°)が入っていた。light-sim-ign の
     台座向きが違っても、xacro を読む方式なので問題なし。別名/別パスの xacro なら
     `-p xacro_path:=... -p srdf_path:=...` で上書き。
2. **/joint_states_filtered**: `joint_state_filter.py` は `*_mimic` を除く。タイヤ版に mimic が
   無くてもフィルタは素通しで動く。`servo.yaml` の `joint_topic` は `/joint_states_filtered` のままでよい。
3. **forward_position_controller の衝突**: 掴むのは `Joint_1..5/position` のみ。走行コントローラ
   (mecanum_drive / diff_drive 等)とは独立なので競合しない。`arm_controller`(JTC)とだけ
   command interface を奪い合うので switch_controllers で切替運用するのは両ブランチ共通。
4. **gz_ros2_control の update_rate / controller spawn**: 既存の bringup がアーム系を spawn して
   いることが前提(arm_controller / joint_state_broadcaster)。タイヤ版 bringup でも同様に
   立ち上がっていることを確認。

---

## F. 既知の経緯(ハマりどころ)
- 5軸 + `position_only_ik:true` は **Servo の Cartesian(twist) では効かない**(KDL姿勢IK側のみ)。
  twist で angular=0 を送ると6×5の過剰拘束になり方向によって動きが鈍る → `servo_keyboard_jog`
  (3×5位置ジョグ)を推奨。
- キーボードは「押してる間だけ」方式だと端末のキーリピート初回待ちで途切れる → **速度保持方式**。
- キーボードノードは **`ros2 launch` だと stdin が届かない** → 必ず `ros2 run` で前面実行。
- 軸90°ずれの真因は「古い `amir_description/urdf/amir_mecanum3.urdf`(base rpy=0)を読んでいた」こと。
  sim と同じ xacro 展開URDF(base rpy=-90°)を使えば補正不要。
