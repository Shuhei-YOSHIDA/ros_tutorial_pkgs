tutorial_pkg3
====

Overview
====
ROSとシミュレータを試す．

How To Use
====

予め例題として用いているturtlebotのシミュレータをインストール

```bash
## (distro)の部分はROSのdistributionに合わせること．ex.kinetic, indigo
$ sudo apt install ros-(distro)-turtlebot-gazebo
$ sudo apt install ros-(distro)-turtlebot-teleop
## 初回起動時にシミュレータ上のオブジェクトの自動ダウンロードがある(かも)
$ roslaunch turtlebot_gazebo turtlebot_world.launch
```

上記のコマンドを実行してシミュレータが立ち上がることを確認すること．

```bash
$ roslaunch tutorial_pkg3 turtle_sim.launch
```
でこのレポジトリでのlaunch ファイルが実行される．

turtlebot を動かしたい場合は，以下のようにキーボードから命令が可能．
```bash
$ roslaunch turtlebot_teleop keyboard_teleop.launch
## 端末にフォーカスを合わせれば，キーボードがコントローラ代わりになる.
```
