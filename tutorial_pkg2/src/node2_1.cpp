/**
 * このnodeはkinectのようなRGBDセンサから得られる値を処理します．
 * またOpenCVやPCLの使用例を含んでいます．
 *
 * reference:
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * http://wiki.ros.org/ja/pcl/Tutorials
 */

// +++++++++++++++++++++++++++++++++
// published topics
// /image_test_lines
// /scan_test_marker
// /points_test_plane_detected

// subscribed topics
// /camera/rgb/image_rect_color @ RGB画像
// /scan @ LRFの距離値
// /camera/depth_registered/points @ 3次元点群

// +++++++++++++++++++++++++++++++++
// memo
// OpenCVとかPCLの使用例を含んだ例です．
// OpenCVではcv::Matという型で画像を扱います．
// なので，ROSのsensor_msgs/Imageとcv::Matは相互変換される必要があります．
// その中で役立つのが，cv_bridgeというパッケージです．
// また，参考ページの中では，sensor_msgs/Imageをpublish, subscribeするとき，
// image_transport パッケージを使って効率的に行っているようです．
// PCLでは．pcl::PointCloud<pcl::PointXYZ>間の
// sensor_msgs/PointCloud2 型を変換が可能です．

#include "ros/ros.h"

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h" // エンコードを意味するリテラル定義

// Point Cloud Library(PCL)のインクルード
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
// PCL の機能群
#include "pcl/segmentation/sac_segmentation.h" // For RANSAC

// LRFのデータ処理表示用にrvizでマーカ情報をtopicに送る．
#include <visualization_msgs/Marker.h>

#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <sstream>

ros::Publisher image_pub, scan_pub, points_pub;
ros::Subscriber image_sub, scan_sub, points_sub;

// topic "/camera/rgb/image_rect_color" に対するコールバック
void image_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr; // 相互変換クラスCvImage型へのポインタ
  try
  {
    // sensor_msgs/ImageをcvImage型に変換
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // OpenCVを使って，画像処理をする．
  // 参考 http://opencv.jp/opencv-2svn/cpp/feature_detection.html
  cv::Mat dst, color_dst;
  cv::Canny(cv_ptr->image, dst, 50, 200, 3); // Canny変換
  cv::cvtColor(dst, color_dst, CV_GRAY2BGR);

  std::vector<cv::Vec4i> lines; // 始点/終点での線分ベクトル
  // 確率的Hough変換を用いて画像中の線分を検出する
  cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10);
  // color_dstに見つけた線を引く．
  for(size_t i = 0; i < lines.size(); i++)
    cv::line(color_dst, cv::Point(lines[i][0], lines[i][1]),
      cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8);

  cv_ptr->image = color_dst;

  // OpenCVの機能で画像表示
  // cv::imshow("image window", cv_ptr->image);
  // cv::waitKey(3);

  // cv_ptrからsensor_msgs/Image を生成して，publish
  // また，その際にheader のタイムスタンプを現在に変更
  cv_ptr->header.stamp = ros::Time::now();
  image_pub.publish(cv_ptr->toImageMsg());
}

// topic /scanに対するコールバック
void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // sensor_msgs/LaserScan 型のLRFの点のデータは
  // 極座標系で表現されています．
  // xyz系に変換して重心を試しに求めてみます．

  float ang_min = msg->angle_min; // 最初の点の方向
  float ang_inc = msg->angle_increment; // 光線の角度差
  float cod[3] = {0,0,0};

  // ROS msg でfloat32[] のような可変配列はc++だとstd::vectorにあたる．
  // sensor_msgs/LaserScan 型の距離データは
  // rangesという要素にfloat32[]として存在する．
  std::vector<float>::const_iterator itr;
  int count = 0;
  for (itr = msg->ranges.begin(); itr != msg->ranges.end(); ++itr)
  {
    float x, y, z;
    // ある角度の距離値が計測不能の時，nanとして記録されている
    // 計測範囲にある値をチェック
    if ((msg->range_min < *itr) && (*itr < msg->range_max))
    {
      x = (*itr)*cos(ang_min + ang_inc * count);
      y = (*itr)*sin(ang_min + ang_inc * count);
      z = 0; // LRFデータはxy座標平面上にあるので．

      // 逐次的に重心計算
      float c = count;
      cod[0] = c/(c+1)*cod[0] + 1/(c+1)*x; // x方向
      cod[1] = c/(c+1)*cod[1] + 1/(c+1)*y; // y方向
      cod[2] = 0; // z方向の重心，常にゼロ
    }

    count++;
  }


  // 重心の座標をマーカーとして表示できるようpublishする．rvizで確認
  visualization_msgs::Marker output;
  output.header = msg->header; // 座標系の名前はROSにとって重要
  output.type = visualization_msgs::Marker::SPHERE;
  output.action = visualization_msgs::Marker::ADD;
  output.scale.x = 0.10; output.scale.y = 0.10; output.scale.z = 0.10;
  output.color.g = 1.0; output.color.a = 0.8;
  // マーカーの座標に重心の座標を設定
  output.pose.position.x = cod[0];
  output.pose.position.y = cod[1];
  output.pose.position.z = cod[2];
  output.pose.orientation.w = 1.0;

  ROS_INFO("x:%f, y:%f, z:%f", cod[0], cod[1], cod[2]);

  scan_pub.publish(output);
}


// topic /camera/depth_registered_pointsに対するコールバック
void points_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // sensor_msgs/PointCloud2 を pcl/PointCloud に変換
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg (*msg, cloud);

  // 3次元点群の中から平面部分を探索する
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  // 平面探索RANSAC
  seg.segment (inliers, coefficients);

  if (inliers.indices.size () == 0)
  {
    ROS_INFO("Could not estimate a planar model for the given dataset.");
    return;
  }


  // 平面上の点を青く染める
  for (size_t i = 0; i < inliers.indices.size (); i++)
  {
    cloud.points[inliers.indices[i]].r = 0;
    cloud.points[inliers.indices[i]].g = 0;
    cloud.points[inliers.indices[i]].b = 255;
  }

  // 平面部分を青く染めた点群をPublishする
  sensor_msgs::PointCloud2 output;
  // pclの型をROSのメッセージ型に変換する
  pcl::toROSMsg (cloud, output);
  output.header.stamp = ros::Time::now();
  points_pub.publish (output);
}

// publisher変数などの初期設定
void Setting(ros::NodeHandle& n)
{
  image_pub = n.advertise<sensor_msgs::Image>(
          "image_test_lines", 10);
  scan_pub = n.advertise<visualization_msgs::Marker>(
          "scan_test_marker", 10);
  points_pub = n.advertise<sensor_msgs::PointCloud2>(
          "points_test_plane_detected", 10);

  image_sub = n.subscribe(
          "/camera/rgb/image_rect_color", 10, image_Callback);
  scan_sub = n.subscribe(
          "/scan", 10, scan_Callback);
  points_sub = n.subscribe(
          "/camera/depth_registered/points", 10, points_Callback);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node2_1");
  ros::NodeHandle n;

  Setting(n);

  ros::spin();
  return 0;
}
