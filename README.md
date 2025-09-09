# humbleble_ws
実機やシミュレーションを動かすときに必要なパッケージたちです。
シミュレーションは実装途中なので、おいおいやっていきます。

## 前提条件
- ros2 humbleがダウンロードされていること
- 研究室のサーバーにある　`AMIR/ros2_basic`　がインストールされており、`source ~/... ` されていること

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

2. ワークスペースをビルド
```bash
cd ~/ros2_humble_ws
colcon build --symlink-install
```


