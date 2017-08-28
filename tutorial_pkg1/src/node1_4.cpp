#include <ros/ros.h>
#include <std_srvs/Empty.h> // Serviceの型のインクルード
#include <std_srvs/SetBool.h> // Serviceの型のインクルード

/**
 * std_srvs/Emptyとstd_srvs/SetBoolのそれぞれの型の
 * ２つのservice呼び出しを行うnode(client)
 * referenece:
 * http://wiki.ros.org/ja/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node1_4");
  ros::NodeHandle n;

  // サービス呼び出しのためのクラス．使うサービス名を渡す．
  ros::ServiceClient client1
      = n.serviceClient<std_srvs::Empty>("service_Empty_example");
  ros::ServiceClient client2
      = n.serviceClient<std_srvs::SetBool>("service_SetBool_example");

  // serviceの型のrequest部分に入力値を入れる
  std_srvs::Empty empty_srv; // Empty型は入出力ともに無い
  std_srvs::SetBool setbool_srv;
  setbool_srv.request.data = true; // SetBool型はbool dataという入力がある

  ROS_INFO("Call 2 services");

  // serviceの呼び出しを掛ける
  if (client1.call(empty_srv)) // server側のservice処理が終わるまで待つ
    ROS_INFO("Success to call service_Empty_example service!");
  else
    ROS_ERROR("Failed to call service_Empty_example!");

  if (client2.call(setbool_srv))
  {
    ROS_INFO("Success to call service_SetBool_example service!");
    // SetBool型はresponse部分に
    // bool success, string messageがある
    std::string message = setbool_srv.response.message;
    if (setbool_srv.response.success)
      ROS_INFO("success:true, message:%s", message.c_str());
    else
      ROS_INFO("success:false, message:%s", message.c_str()); 
  }
  else
    ROS_ERROR("Failed to call service_SetBool_example!");

  return 0;
}


