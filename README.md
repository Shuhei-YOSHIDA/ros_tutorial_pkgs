# tutorial_pkgs_20170907
このレポジトリはROSのチュートリアルです．


# 各パッケージについて
このレポジトリにあるROSのパッケージは
* tutorial_pkg1
* tutorial_pkg2
...となっています.

## tutorial_pkg1
このパッケージでは， ...

## tutorial_pkg2
このパッケージでは， ...

# How to Use
動作確認をしているROSのディストリビューションは，
* kinetic (master branch)
* indigo  (indigo branch)
です．

## コードの準備とレポジトリのビルド
まず，ROSのワークスペースを作ります．
すでにどこかで作ったことがあるかもしれません．
その場合はそのワークスペースを使ってもおそらく問題は無いでしょう．

次にこのレポジトリをクローンします．
```bash
your_ws/src$ git clone url
```
をしてください．indigo を使っている場合は，
```bash
your_ws/src$ git checkout indigo
```
をして，indigoのコードにしましょう．

レポジトリをクローンして，コードの準備ができたなら，ビルドをしましょう．
使っているワークスペースのルートディレクトリで次のコマンドを実行してください．
```bash
your_ws$ catkin_make
```

また，使っているワークスペースの環境変数を設定するために，
```bash
$ source  your_ws/devel/setup.bash
```
を実行してください．
端末を新しく開くたびにこのコマンドを実行するのが面倒な場合は，
~/.bashrc に記述しておくと良いでしょう.

## レポジトリ内のパッケージ内のROSノードを実行するには
ROSのノードを単体で実行するには，
```bash
$ rosrun パッケージ名 ノードの実行ファイル名
$ rosrun tutorial_pkg1 sample.py   ##pythonならスクリプト名
$ rosrun tutorial_pkg1 sample_node ##C++ならCMakeList.txtで設定
```
のようにrosrunというコマンドを使って実行します．
このコマンドを使うには環境変数が読み込まれてい必要があります．
`rosrun tutoria` あたりでtabキーを叩いても補完されないなら，
your_ws/devel/setup.bashをきちんとsourceしているかを確認してください．

```bash
$ env | grep ROS_PACKAGE_PATH
```
で環境変数に自分のワークスペースが読み込まれているのを確認しても良いでしょう．
