#include <ros/ros.h>
#include <actionlib/TestAction.h> // Action の型
#include <actionlib/client/simple_action_client.h>
/**
 * このnodeはactionlib/Test 型のaction
 * reference:
 * http://wiki.ros.org/ja/actionlib
 */

// actionlib/Test は
// int32 goal
// ---
// int32 result
// ---
// int32 feedbackの形式

typedef actionlib::SimpleActionClient<actionlib::TestAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node1_8");
  Client client("action_test", true); // true -> don't need ros::spin()

  // Action server nodeが立ち上がるのを待つ
  client.waitForServer();

  actionlib::TestGoal goal;
  // goalの設定
  goal.goal = 10;

  // serverにactionのgoalを送る
  client.sendGoal(goal);

  // server の終了を引数の時間まで待つ
  client.waitForResult(ros::Duration(15.0));

  // serverの処理の結果を確認する
  ROS_INFO("State: %s", client.getState().toString().c_str());

  return 0;
}
