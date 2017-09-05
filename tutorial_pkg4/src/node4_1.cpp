/**
 * tutorial_pkg2 パッケージでのnode2_1では
 * LRF計測値のTopic /scanの内容を読んでデータの図心を
 * 計算して，RVizのMarker情報としてPublishしていました．
 * その時の図心の座標系はLRFデータと同じものでしたが，
 * もしtopicのヘッダ情報にある座標系名を変更したら，
 * RVizで表示される位置はずれるはずです.
 * ただ，TFを用いて正しい座標位置に変換してPublishをすれば，
 * RVizで表示されるときに再び座標変換が行われて，
 * 同じ位置に表示されるはずです．
 *
 * tutorial_pkg3 のシミュレータと
 * tutorial_pkg2 のNodeが動いている状態で
 * このNodeを試してみましょう．
 *
 * reference:
 * http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29
 */

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// topic /scanから図心を計算 node2_1と同じ
visualization_msgs::Marker calcCenter(
        const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float ang_min = msg->angle_min;
  float ang_inc = msg->angle_increment;
  float cod[3] = {0,0,0};

  std::vector<float>::const_iterator itr;
  int count = 0;
  for (itr = msg->ranges.begin(); itr != msg->ranges.end(); ++itr)
  {
    float x, y, z;
    if ((msg->range_min < *itr) && (*itr < msg->range_max))
    {
      x = (*itr)*cos(ang_min + ang_inc * count);
      y = (*itr)*sin(ang_min + ang_inc * count);
      z = 0;

      // 逐次的に重心計算
      float c = count;
      cod[0] = c/(c+1)*cod[0] + 1/(c+1)*x; // x方向
      cod[1] = c/(c+1)*cod[1] + 1/(c+1)*y; // y方向
      cod[2] = 0; // z方向の重心，常にゼロ
    }
    count++;
  }

  visualization_msgs::Marker output;
  // 座標系の名前はセンサの座標系にとりあえず合わせる
  output.header = msg->header;
  output.type = visualization_msgs::Marker::SPHERE;
  output.action = visualization_msgs::Marker::ADD;
  output.scale.x = 0.10; output.scale.y = 0.10; output.scale.z = 0.10;
  // マーカーの座標に重心の座標を設定
  output.pose.position.x = cod[0];
  output.pose.position.y = cod[1];
  output.pose.position.z = cod[2];
  output.pose.orientation.w = 1.0;

  return output;
}

class scan_process
{
private:
  void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  // Buffer変数の寿命に注意すること
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  ros::Publisher scan_pub_correct;
  ros::Publisher scan_pub_incorrect;
  ros::Subscriber scan_sub;

public:
  scan_process(ros::NodeHandle &n);
};

scan_process::scan_process(ros::NodeHandle &n) : tfListener(tfBuffer)
{
  scan_sub = n.subscribe("/scan",
          10, &scan_process::scan_Callback, this);
  scan_pub_incorrect = n.advertise<visualization_msgs::Marker>(
          "center_marker_incorrect", 10);
  scan_pub_correct = n.advertise<visualization_msgs::Marker>(
          "center_marker_correct", 10);
}

void scan_process::scan_Callback(
        const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // node2_1と同様に/scanより図心を計算する
  visualization_msgs::Marker marker_msg = calcCenter(msg);

  // 元のセンサ座標系の名前を端末に表示
  std::string sensor_coord_name = marker_msg.header.frame_id;
  // "/name" の形式．最初のスラッシュを削除
  sensor_coord_name.erase(sensor_coord_name.begin());
  ROS_INFO_THROTTLE(60,
          "name of coord is %s", sensor_coord_name.c_str());

  // 座標系をbase_linkに変換したい
  std::string coord_name = "base_link";

  // 座標系の名前だけ変えた不適切なもの
  visualization_msgs::Marker incorrect_msg = marker_msg;
  incorrect_msg.header.frame_id = coord_name;
  incorrect_msg.color.r = 1.0; // markerの色を赤に
  incorrect_msg.color.a = 0.8;

  // TFを用いてきちんと座標変換を行った図心位置のMarker
  // センサ座標系とcoord_nameでの座標系での変換を/tfより取得する
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    // TFの座標系変換を取得するときは本来はその取得時刻に注意すべき
    // 詳しくは"tf,tf2完全理解"で検索
    transform_stamped = tfBuffer.lookupTransform(
          coord_name,   // target frame
          sensor_coord_name,          // source frame
          ros::Time(0),        // 変換可能なバッファのうち最新のもの
          ros::Duration(0.5)   // tfでのwaitTransformがここに含まれている
          );
    //std::cout << transform_stamped << std::endl;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  // 座標系変換の値を直接自分で使うこともできるが，
  // tf2::doTransform でデータの座標変換計算可能.
  // ただしテンプレート関数になっていて，
  // 特定の型についてそれぞれのパッケージで実装されている
  // EX. tf2_geometry_msgs
  geometry_msgs::PoseStamped t_in, t_out;
  t_in.header = marker_msg.header;
  t_in.pose = marker_msg.pose;
  tf2::doTransform(t_in, t_out, transform_stamped);

  visualization_msgs::Marker correct_msg = marker_msg;
  correct_msg.header.frame_id = coord_name;
  correct_msg.pose = t_out.pose; // coord_name に対して正しい座標
  correct_msg.color.b = 1.0; // markerの色を青に
  correct_msg.color.a = 0.8;

  scan_pub_incorrect.publish(incorrect_msg);
  scan_pub_correct.publish(correct_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node4_1");
  ros::NodeHandle n;

  scan_process process(n);
  ros::spin();

  return 0;
}
