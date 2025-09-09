# humbleble_ws
実機やシミュレーションを動かすときに必要なパッケージたちです。
シミュレーションは実装途中なので、おいおいやっていきます。
またmoveit2を動かすときに、urdfが読み込めなくて実行できないときがあるかもしれません。

##インストール！！


```bash
mkdir -p ~/ros2_humble_ws/src
cd ~/ros2_humble_ws/src
git clone 

rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

```
