#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * 単純に一つのtopicをsubscribeするnode
 * reference:
http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */

// 指定のtopicが得られた時呼び出されるコールバック関数
// topicを通して他のnodeから得られたデータを処理する．
// 引数のmsgに購読されたデータが入っている
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  // ROS_INFOはlog出力用のマクロ．端末画面とROSのlogファイルに出力
  ROS_INFO("Got next message --> %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node1_2");
  ros::NodeHandle n;

  // ros::Subscriberはtopicの購読とその時発生させるコールバック関数を
  // 設定するために必要なクラス．
  // 引数に，topic名とデータキュー，その時の関数ポインタを渡す．
  ros::Subscriber s_data_sub = n.subscribe("s_data", 1, chatterCallback);

  // コールバックの待受状態になる．
  // どこかのnodeから購読対象になっているtopic がpublishされると，
  // ros::Subscriberに設定したコールバック関数が呼ばれる．
  // nodeが止められるまで，待ち受け状態は続く
  ros::spin();

  return 0;
}
