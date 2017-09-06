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

Description
====
* sensor_msgs/PointCloud2 型 (3次元点群)
* sensor_msgs/LaserScan 型 (Laser Range Finder から得られるデータ)
* sensor_msgs/Image 型 (画像)

上記のデータを処理する例を示す．

## src/node2_1.cpp
* 3次元点群から平面検出
* LRFのデータを読み取り，その重心を計算
* 画像からHough変換を用いて直線を抽出する
