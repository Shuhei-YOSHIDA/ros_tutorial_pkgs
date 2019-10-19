/**
 * @file node5_1.cpp
 * @brief Message filters sample
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>

#include <message_filters/time_synchronizer.h>

#include <message_filters/time_sequencer.h>

#include <message_filters/cache.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/chain.h>


using namespace geometry_msgs;

void callback1(const PoseStampedConstPtr& msg11, const PoseStampedConstPtr& msg12)
{
  ROS_INFO_STREAM("callback1:" << "msg11 " << msg11->header
                               << "msg12 " << msg12->header);
}

void callback2(const PoseStampedConstPtr& msg21)
{
  ROS_INFO_STREAM("callback2:" << "msg21 " << msg21->header);
}

void callback3(const PoseStampedConstPtr& msg31)
{
  ROS_INFO_STREAM("callback3:" << "msg31 " << msg31->header);
}

void callback4(const PoseStampedConstPtr& msg41, const PoseStampedConstPtr& msg42)
{
  ROS_INFO_STREAM("callback4:" << "msg41 " << msg41->header
                               << "msg42 " << msg42->header);
}

void callback5(const PoseStampedConstPtr& msg51)
{
  ROS_INFO_STREAM("callback5:" << "msg51 " << msg51->header);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node5_1");
  ros::NodeHandle nh;

  // Time Synchronizer
  // Topicのtimestampが完全に一致した時にのみcallbackを呼ぶ
  message_filters::Subscriber<PoseStamped> sub_11(nh, "topic_pkg5_11", 1);
  message_filters::Subscriber<PoseStamped> sub_12(nh, "topic_pkg5_12", 1);
  message_filters::TimeSynchronizer<PoseStamped, PoseStamped> sync(sub_11, sub_12, 10);
  sync.registerCallback(boost::bind(&callback1, _1, _2));

  // Time Sequencer (single-in/single-output  (simple) filter)
  // Topicのheaderのseqが示す順番通りにcallbackを呼ぶ
  message_filters::Subscriber<PoseStamped> sub_21(nh, "topic_pkg5_21", 1);
  message_filters::TimeSequencer<PoseStamped> seq(sub_21, ros::Duration(0.1), ros::Duration(0.01), 10);
  seq.registerCallback(callback2);

  // Cache (single-in/single-output  (simple) filter)
  // 複数のメッセージをNまでキャッシュして，最新のメッセージに対してcallbackを呼ぶ．
  message_filters::Subscriber<PoseStamped> sub_31(nh, "topic_pkg5_31", 1);
  message_filters::Cache<PoseStamped> cache(sub_31, 30);
  cache.registerCallback(callback3);

  // Policy-Based Synchronizer
  // 複数のTopicを指定したPolicy に基づいてフィルタしてcallbackを呼ぶ．
  message_filters::Subscriber<PoseStamped> sub_41(nh, "topic_pkg5_41", 1);
  message_filters::Subscriber<PoseStamped> sub_42(nh, "topic_pkg5_42", 1);
  // sync_policies::ExactTime is almost same as TimeSynchronizer
  int queue_size = 10;
  message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<PoseStamped, PoseStamped>>
      sync_a(
        message_filters::sync_policies::ApproximateTime<PoseStamped, PoseStamped>(queue_size),
        sub_41,
        sub_42);
  sync_a.registerCallback(boost::bind(&callback4, _1, _2));

  // Chain
  // 複数のsimple filterの出力をまとめて，その出力に対応してcallbackを呼ぶ．
  message_filters::Subscriber<PoseStamped> sub_51(nh, "topic_pkg5_51", 1);
  message_filters::Chain<PoseStamped> c;
  c.addFilter(&sub_51);
  c.addFilter(&seq);
  c.addFilter(&cache);
  c.registerCallback(callback5);

  ros::spin();

  return 0;
}
