/**
 * @file test_publisher.cpp
 * @brief Publish topics to message filters' node
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace geometry_msgs;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;

  // For Time Synchoronizer
  auto pub1_1 = nh.advertise<PoseStamped>("topic1_1", 1);
  auto pub1_2 = nh.advertise<PoseStamped>("topic1_2", 1);

  // For Time Sequencer
  auto pub2_1 = nh.advertise<PoseStamped>("topic2_1", 1);
  auto pub2_2 = nh.advertise<PoseStamped>("topic2_2", 1);

  // For Cache
  auto pub3_1 = nh.advertise<PoseStamped>("topic3_1", 1);
  auto pub3_2 = nh.advertise<PoseStamped>("topic3_2", 1);

  // For Policy-Based Synchronizer
  auto pub4_1 = nh.advertise<PoseStamped>("topic4_1", 1);
  auto pub4_2 = nh.advertise<PoseStamped>("topic4_2", 1);

  // For Chain
  auto pub5_1 = nh.advertise<PoseStamped>("topic5_1", 1);
  auto pub5_2 = nh.advertise<PoseStamped>("topic5_2", 1);

  ros::Rate loop(1);
  while (ros::ok())
  {
    // Publish each topics with delays
    auto stamp = ros::Time::now();

    loop.sleep();
  }

  return 0;
}
