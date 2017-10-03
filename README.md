[![Build Status](https://travis-ci.org/Shuhei-YOSHIDA/ros_tutorial_pkgs.svg?branch=master)](https://travis-ci.org/Shuhei-YOSHIDA/ros_tutorial_pkgs)
# ros_tutorial_pkgs
このレポジトリはROSのチュートリアルです．


# 各パッケージについて
このレポジトリにあるROSのパッケージは
* tutorial_pkg1
* tutorial_pkg2
...となっています.

## tutorial_pkg1
このパッケージでは，
ROSのNode, Topic, Service, msg, srv, parameter, actionlibといった基本の概念の説明をするためのコードが置かれています．

## tutorial_pkg2
このパッケージでは，KinectのようなRGBDセンサーから得られる情報をOpenCVやPointCloudLibrary で処理する方法について例を挙げています．

## tutorial_pkg3
このパッケージでは，tutorial_pkg2のNodeをTurtlebotのシミュレーション上で使っています．

## tutorial_pkg4
このパッケージでは，tutorial_pkg2のNodeがPublishするTopic を用いて，tf による座標変換の例を示しています．

# How to Use
動作確認をしているROSのディストリビューションは，
* kinetic
* indigo
です．
ただ，現在メインはkinetic としています．

Turtlebotのシミュレータを以下のコマンドで予めインストールしてください．
```bash
## (distro)の部分はROSのdistributionに合わせること．ex.kinetic, indigo
$ sudo apt install ros-(distro)-turtlebot-gazebo
$ sudo apt install ros-(distro)-turtlebot-teleop
## 初回起動時にシミュレータ"Gazebo"がネットワーク経由でCADデータを(おそらく)自動ダウンロードするのでやっておきましょう
$ roslaunch turtlebot_gazebo turtlebot_world.launch
```

また，次のfreenectをインストールしておくと，実機のkinect などが使えます．
```bash
$ sudo apt install ros-(distro)-freenect-stack
```

## コードの準備とレポジトリのビルド
まず，ROSのワークスペースを作ります．
すでにどこかで作ったことがあるかもしれません．
その場合はそのワークスペースを使ってもおそらく問題は無いでしょう．
以下の説明では，ホームディレクトリに`~/your_ws`というワークスペースがあるという前提とします．

次にこのレポジトリをワークスペースのsrcディレクトリにクローンします．
下記のコマンド,
```bash
~/your_ws/src$ git clone https://github.com/Shuhei-YOSHIDA/ros_tutorial_pkgs.git
```
を実行してください．

レポジトリをクローンして，コードの準備ができたなら，ビルドをしましょう．
使っているワークスペースのルートディレクトリで次のコマンドを実行してください．
```bash
~/your_ws/src$ cd ..
~/your_ws$ catkin_make
```

また，使っているワークスペースの環境変数を設定するために，
```bash
$ source  ~/your_ws/devel/setup.bash
```
を実行してください．
端末を新しく開くたびにこのコマンドを実行するのが面倒な場合は，
~/.bashrc の末尾にこの一行を記述しておくと良いでしょう.

## レポジトリ内のパッケージ内のROSノードを実行するには
ROSのノードを単体で実行するには，`roscore`を実行した状態で，
```bash
$ rosrun パッケージ名 ノードの実行ファイル名
$ rosrun tutorial_pkg1 sample.py   ##pythonならスクリプト名
$ rosrun tutorial_pkg1 sample_node ##C++ならcmakelist.txtで設定
```
のようにrosrunというコマンドを使って実行します．
このコマンドを使うには環境変数が読み込まれている必要があります．
`rosrun tutoria` あたりでtabキーを叩いても補完されないなら，
~/your_ws/devel/setup.bashをきちんとsourceしているかを確認してください．

```bash
$ env | grep ROS_PACKAGE_PATH
```
で環境変数に自分のワークスペースが読み込まれているのを確認しても良いでしょう．
