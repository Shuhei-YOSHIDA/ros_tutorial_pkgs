tutorial_pkg2
====

Overview
====
センサから得られたデータをROSを通して処理する例

How to Use
====
kinect実機などを使う場合は以下のようにfreenectを入れておくと良いでしょう

```bash
$ sudo apt install ros-(distro)-freenect-stack
```

KinectのようなセンサからPublishされる画像や点群のtopicの名前が
このtutorial_pkg2 の ノードでSubscribeしている名前と合わない場合は，
launchファイルの<remap>タグを使ったり，以下のようにrosrunでtopic名を
変更して実行してみてください．

```bash
$ rosrun tutorial_pkg2 tutorial_pkg2_node_1 /camera/depth_registered/points:=/camera/depth/points
```

Description
====
* sensor_msgs/PointCloud2 型 (3次元点群)
* sensor_msgs/LaserScan 型 (laser range finder から得られるデータ)
* sensor_msgs/Image 型 (画像)

上記のデータを処理する例を示す．

## src/node2_1.cpp
* 3次元点群から平面検出
* LRFのデータを読み取り，その重心を計算
* 画像からHough変換を用いて直線を抽出する

## src/node2_2.cpp
* sensor_msgs/PointCloud2型を扱う
* PointCloud2型は様々なデータ型(XYZ, XYZRGB, 反射強度，法線ベクトル)に対応できるようになっているが，直接データを扱うのが面倒であるためPCLの型に変換してから各要素にアクセスする例を示す．
