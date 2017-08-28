#include <ros/ros.h>
#include <std_srvs/Empty.h> // Serviceの型のインクルード
#include <std_srvs/SetBool.h> // Serviceの型のインクルード

/**
 * std_srvs/Emptyとstd_srvs/SetBoolのそれぞれの型による
 * ２つのserviceを持つnode(server側のnode)
 * referenece:
 * http://wiki.ros.org/ja/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
 */

// std_srvs/Empty 型によるサービス実装例
// client側から呼び出された時，ros::ServiceServerに設定された
// この関数が用いられる．
// 引数には.srvファイルの前半(入力側)がRequest, 
// 後半(出力側)がResponseとして与えられる．
bool empty_example(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res)
{
  // std_srvs/Emptyは入力も出力も無い
  ROS_INFO("service_Empty_example is called!");
  return true; //終了時にtrueを返す
}

bool setbool_example(std_srvs::SetBool::Request &req,
                     std_srvs::SetBool::Response &res)
{
  ROS_INFO("service_SetBool_example is called!");
  if (req.data) // SetBoolは入力にbool型のdataという変数を持つ
  {
    // SetBoolは出力にbool型のsuccess, string型のmessageという変数を持つ
    res.success = true;
    res.message = "data was true";
  }
  else
  {
    res.success = true;
    res.message = "data was false";
  }
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "node1_3");
  ros::NodeHandle n;

  // ros::ServiceServer はサービス名と関数ポインタを受け取って
  // serviceを設定するクラス．
  ros::ServiceServer service1
      = n.advertiseService("service_Empty_example", empty_example);
  ros::ServiceServer service2
      = n.advertiseService("service_SetBool_example", setbool_example);

  // clientからサービスの呼び出しがあるのを待つ．
  ros::spin();

  return 0;
}

