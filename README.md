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

rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

```

2. ワークスペースをビルド
