tutorial_pkg1
====

Overview
====
ROSのnode, topicなど基本事項に対するチュートリアルのためのコード群

Description
====
このパッケージのnode群

## src/node1_1.cpp
std_msgs/String 型のtopic "s_data"を周期的にpublishするnode"node1_1"

## src/node1_2.cpp
std_msgs/String 型のtopic "s_data"をsubscribeするnode"node1_2"

## scripts/node1_1.py
src/node1_1.cpp をpythonで書き換えたもの

## scripts/node1_2.py
src/node1_2.cpp をpythonで書き換えたもの
