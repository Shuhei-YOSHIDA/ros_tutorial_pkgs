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
  auto pub1_1 = nh.advertise<PoseStamped>("topic_pkg5_11", 1);
  auto pub1_2 = nh.advertise<PoseStamped>("topic_pkg5_12", 1);

  // For Time Sequencer
  auto pub2_1 = nh.advertise<PoseStamped>("topic_pkg5_21", 1);

  // For Cache
  auto pub3_1 = nh.advertise<PoseStamped>("topic_pkg5_31", 1);

  // For Policy-Based Synchronizer
  auto pub4_1 = nh.advertise<PoseStamped>("topic_pkg5_41", 1);
  auto pub4_2 = nh.advertise<PoseStamped>("topic_pkg5_42", 1);

  // For Chain
  auto pub5_1 = nh.advertise<PoseStamped>("topic_pkg5_51", 1);

  ros::Rate loop(1);
  while (ros::ok())
  {
    // Publish each topics with delays
    auto stamp = ros::Time::now();

    loop.sleep();
  }

  return 0;
}
