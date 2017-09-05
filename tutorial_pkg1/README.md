tutorial_pkg1
====

Overview
====
ROSのnode, topicなど基本事項に対するチュートリアルのためのコード群

Description
====
**このパッケージのnode群**

## src/node1_1.cpp
std_msgs/String 型のtopic "s_data"を周期的にpublishするnode "node1_1"

## src/node1_2.cpp
std_msgs/String 型のtopic "s_data"をsubscribeするnode "node1_2"

## scripts/node1_1.py
src/node1_1.cpp をpythonで書き換えたもの

## scripts/node1_2.py
src/node1_2.cpp をpythonで書き換えたもの

## src/node1_3.cpp
std_srvs/Empty 型のservice "service_Empty_example" と
std_srvs/SetBool 型のservice "service_SetBool_example" を実行する
node "node1_3"

## src/node1_4.cpp
std_srvs/Empty 型のservice "service_Empty_example" と
std_srvs/SetBool 型のservice "service_SetBool_example" を呼び出す
node "node1_4"

## src/node1_5.cpp
std_msgs/UInt16 型のtopic "num_data" をsubscribeして，
それに応じてstd_msgs/String 型のtopic "s_data"をpublishするnode.
また，std_srvs/Trigger 型のservice "switch_message"を実行する．

## src/node1_6.cpp
ROS parameter を読み出すnode. string型の"param1" と，
float型の"param2"をパラメータサーバから読み出して端末に表示する．
また，"int_param", "string_param", "bool_param"という名前のparameterを
パラメータサーバに登録する．

## src/node1_7.cpp
actionlib/TestAction 型に基づくaction server

## src/node1_8.cpp
action server "node1_7"に命令するaction client


**このパッケージで定義されたmsg**

## msg/motor_data.msg
例として，モータの名前，最大印加電圧，電源状態のデータを含むmsgを定義した


**このパッケージで定義されたsrv**

## srv/move_motor.srv
例として，tutorial\_pkg1/motor_data 型のデータと目標位置を渡して，
その可否とメッセージが返却されるようなsrvを定義した

