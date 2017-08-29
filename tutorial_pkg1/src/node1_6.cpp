#include <ros/ros.h>
/**
 * このnodeは，parameterを読み出して表示する
 * 端末で $ rosparam set パラメータ名 値
 * でパラメータサーバにparamterを登録できる．
 * また，node1_5_param というパラメータを設定する
 * reference:
 * http://wiki.ros.org/roscpp/Overview/Parameter%20Server
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node1_6");
  ros::NodeHandle n;

  std::string param1_value;
  if (n.getParam("param1", param1_value))
  {
    std::string message ="parameter:param1 is gotten, %s";
    ROS_INFO(message.c_str(), param1_value.c_str());
  }
  else
  {
    ROS_ERROR("parameter:param1 couldn't be gotten");
  }

  double param2_value;
  if (n.getParam("param2", param2_value))
  {
    std::string message ="parameter:param2 is gotten, %f";
    ROS_INFO(message.c_str(), param2_value);
  }
  else
  {
    ROS_ERROR("parameter:param2 couldn't be gotten");
  }

  // parameter をパラメータサーバに設定する
  n.setParam("int_param", 10);
  n.setParam("string_param", "string_data");
  n.setParam("bool_param", false);
  return 0;
}

