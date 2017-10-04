/**
 * このノードは点群の型としてROSでデファクトスタンダードな
 * sensor_msgs/PointCloud2 の処理を示します．
 *
 * reference:
 * http://docs.pointclouds.org/1.5.1/common_2include_2pcl_2common_2io_8h.html
 */

// +++++++++++++++++++++++++++++++++
// published topics
// visualization_msgs/Marker points_test_marker @ Rvizでのマーカ

// subscribed topics
// sensor_msgs/PointCloud2 /camera/depth_registered/points @ 3次元点群

#include "ros/ros.h"

// Point Cloud Library(PCL)のインクルード
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

// LRFのデータ処理表示用にrvizでマーカ情報をtopicに送る．
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>

ros::Publisher marker_pub;
ros::Subscriber points_sub;

// topic /camera/depth_registered_pointsに対するコールバック
void points_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // sensor_msgs/PointCloud2 を pcl/PointCloud に変換
  //pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*msg, cloud);
  // pcl::PointXYZ, PointXYZRGB以外にも
  // 法線ベクトルや強度を含む場合があります．
  // sensor_msgs/PointCloud2は様々なデータフィールドを持つことができるので
  // 扱う際は $ rostopic echo -n1 /points | grep -v "^data" などを使って，
  // どのようなデータになっているか確認しましょう．

  // 元のmsgのfieldと比べてみよう
  std::string fields = pcl::getFieldsList(*msg);
  std::cout << "1. " << fields << std::endl;
  std::string fields2 = pcl::getFieldsList(cloud);
  std::cout << "2. " << fields2 << std::endl;

  // PointCloudのデータの各点を読んで，図心を計算．
  double cod[3] = {0, 0, 0};
  for (int i = 0; i < cloud.points.size(); i++)
  {
    // 各点に対して計測不可能等の理由でNANが入っていないかチェック
    if (!pcl::isFinite(cloud.points[i])) continue;

    // 図心の計算
    float c = i;
    cod[0] = c/(c+1)*cod[0] + 1/(c+1)*cloud.points[i].x; // x方向
    cod[1] = c/(c+1)*cod[1] + 1/(c+1)*cloud.points[i].y; // y方向
    cod[2] = c/(c+1)*cod[2] + 1/(c+1)*cloud.points[i].z; // z方向
  }

  // 図心の座標をマーカーとして表示できるようpublishする．rvizで確認
  visualization_msgs::Marker output;
  output.header = msg->header; // 座標系の名前はROSにとって重要
  output.type = visualization_msgs::Marker::SPHERE;
  output.action = visualization_msgs::Marker::ADD;
  output.scale.x = 0.10; output.scale.y = 0.10; output.scale.z = 0.10;
  output.color.g = 1.0; output.color.a = 0.8;
  // マーカーの座標に図心の座標を設定
  output.pose.position.x = cod[0];
  output.pose.position.y = cod[1];
  output.pose.position.z = cod[2];
  output.pose.orientation.w = 1.0;

  // 重心の表示
  ROS_INFO("x:%f, y:%f, z:%f", cod[0], cod[1], cod[2]);

  marker_pub.publish(output);
}

// publisher変数などの初期設定
void Setting(ros::NodeHandle& n)
{
  marker_pub = n.advertise<visualization_msgs::Marker>(
          "points_test_marker", 10);

  points_sub = n.subscribe(
          "/camera/depth_registered/points", 10, points_Callback);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node2_2");
  ros::NodeHandle n;

  Setting(n);

  ros::spin();
  return 0;
}
