#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 単純に一つのtopicをsubscribeするnode
# reference:
# http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28python%29
import rospy
from std_msgs.msg import String

# topicが購読されたときに呼ばれるコールバック関数
def callback(data):
    # 端末画面とlogに出力
    rospy.loginfo("Got next message --> %s", data.data)

# nodeの実際の内容
def listener():
    # nodeに名前を設定
    rospy.init_node('node1_2')

    # 購読するtopicと型，その時のコールバック関数の設定
    rospy.Subscriber("s_data", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # 購読しているtopicの待受状態になる．
    rospy.spin()

if __name__ == '__main__':
    listener()
