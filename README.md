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

2. ワークスペースをビルド
