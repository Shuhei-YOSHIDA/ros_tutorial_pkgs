#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Trigger.h>

/**
 * このnodeは，
 * topic "num_data"をsubscribeするとtopic "num_data"を
 * publishする．また，service "switch_message"を持つ．
 *
 * # Topic
 * ## Publish
 * std_msgs/String s_data
 *
 * ## Subscribe
 * std_msgs/UInt16 num_data
 *
 * # Service
 * std_srvs/Trigger switch_message
 */

// ros::Publisher などの変数をまとめるクラス
class node1_5
{
private:
  ros::NodeHandle _n;
  ros::Publisher _s_pub;
  ros::Subscriber _num_sub;
  ros::ServiceServer _switch_srv;

  std::string _message_set[2];
  unsigned int _message_index;

  // topic num_dataを得た後，それに応じてtopic s_dataをpublishする
  void callback(const std_msgs::UInt16::ConstPtr &num_msg)
  {
    int num = num_msg->data;
    std::string message = "MESSAGE: ";

    for (int i = 0; i < num; i++)
      message += _message_set[_message_index];

    // メッセージの型にデータを代入
    std_msgs::String s_msg;
    s_msg.data = message;

    // s_data topicにpublishする．
    _s_pub.publish(s_msg);
  }

  bool switchMessage(std_srvs::Trigger::Request &req,
                     std_srvs::Trigger::Response &res)
  {
    // std_srvs/Triggerはresponse部分のみ
    // bool success, string message だけ
    _message_index = (_message_index == 0 ? 1 : 0);
    res.success = true;
    res.message = "Message is set to " + _message_set[_message_index];
  }

public:
  node1_5()
  {
    // topic, serviceの設定
    // クラスのメソッドをコールバックに登録する際の作法に注意
    _s_pub = _n.advertise<std_msgs::String>("s_data", 1);
    _num_sub = _n.subscribe("num_data", 1, &node1_5::callback, this);
    _switch_srv = _n.advertiseService("switch_message", &node1_5::switchMessage, this);

    _message_set[0] = "(=^･x･^=) < nyan ";
    _message_set[1] = "U・x・U < wang ";
    _message_index = 0; //_message_set の添字
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node1_5");

  // publisherの設定などはコンストラクタで行われる
  node1_5 node;

  // topicやserviceの待受状態になる
  ros::spin();

  return 0;
}

