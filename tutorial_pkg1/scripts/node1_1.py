#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 単純に一つのtopicをpublishするnode
# reference:
# http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# rospyはROSのシステムを使用するのに必須
# Publisherの登録などに用いる．
import rospy
# ROSのメッセージファイルはpythonのモジュールに変換される
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('s_data', String, queue_size=1)
    rospy.init_node('node1_1', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        str = "string data, Hello World!"
        pub.publish(str)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
