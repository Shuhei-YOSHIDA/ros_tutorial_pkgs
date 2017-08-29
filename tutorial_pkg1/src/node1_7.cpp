#include <ros/ros.h>
#include <actionlib/TestAction.h> // Action の型
#include <actionlib/server/simple_action_server.h>
/**
 * このnodeはactionlib/Test 型のaction server
 * actionlibパッケージのTest.actionファイルに従う.
 * reference:kkljj
 * http://wiki.ros.org/ja/actionlib
 */

// actionlib/Test は
// int32 goal
// ---
// int32 result
// ---
// int32 feedbackの形式

typedef actionlib::SimpleActionServer<actionlib::TestAction> Server;

void execute(const actionlib::TestGoalConstPtr& goal, Server* as)
{
  // 1秒毎にデータ表示をgoalの数だけ実行
  // 現在の回数をfeedbackにする
  ros::Rate loop_rate(1);
  actionlib::TestFeedback feedback;
  actionlib::TestResult result;

  // 最大15秒までにしておく
  int num = (goal->goal < 15) ? goal->goal : 15;
  for (int i = 0; i < num; i++)
  {
    // action server にpreemptが送られているかチェック
    if (as->isPreemptRequested() || !ros::ok())
    {
      //action serverの状態をpreemptedにする
      as->setPreempted();
      //as->setPreempted(result);
      return;
    }
    ROS_INFO("action server works... %d ", i);

    // feedbackの内容をTestActionFeedback topicにpublish
    feedback.feedback = i;
    as->publishFeedback(feedback);

    // resultに現在の数を入れておく
    result.result = i;
    loop_rate.sleep();
  }

  // action serverの状態をSucceededにして，actionのresult部分を返す
  as->setSucceeded(result);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node1_7");
  ros::NodeHandle n;
  Server server(n, "action_test", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
