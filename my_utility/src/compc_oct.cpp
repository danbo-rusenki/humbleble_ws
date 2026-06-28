// =============================================================================
//  compc_oct : PointCloud2 → OctoMap 変換ノード
//
//  ★★★ 改修: 点群を TF で target_frame(既定 base_footprint) に動的変換してから
//      OctoMap へ蓄積する ★★★
//   - 入力点群はカメラ光学フレーム(d435_depth_optical_frame)で届くため、
//     そのまま蓄積するとアーム/カメラが動くたびに地図がズレる(smearing)。
//   - 各フレームで TF (cloud_frame → target_frame) を引いて点群を変換し、
//     ロボット基準(base_footprint)に固定した地図を作る。
//     → アームを動かしても (台車が静止していれば) 地図は崩れない。
//   - sensor_origin も target_frame でのカメラ位置に変換するので、
//     insertPointCloud のレイキャスト(free/occupied 判定)も正しくなる。
//
//  パラメータ:
//    target_frame (string, 既定 "base_footprint") : 蓄積する固定フレーム
//                 ※台車も含めワールド固定にしたいなら "odom" を指定
//    resolution   (double, 既定 0.05)            : ボクセルサイズ[m]
//    max_range    (double, 既定 2.5)             : センサからの採用範囲[m] (各軸±)
//    tf_timeout   (double, 既定 0.1)             : TF 待ち時間[s]
// =============================================================================
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

class PcToOctomapNode : public rclcpp::Node
{
public:
  PcToOctomapNode()
  : Node("pc_to_octomap_node"),
    octree_(this->declare_parameter<double>("resolution", 0.05))
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_footprint");
    max_range_    = this->declare_parameter<double>("max_range", 2.5);
    tf_timeout_   = this->declare_parameter<double>("tf_timeout", 0.1);
    // ★ ソースフレーム上書き:
    //   gz の /d435/points は gz規約(X前方/Z上)のデータなのに frame_id だけ
    //   optical(Z前方)が付いている不整合がある。空でなければ msg->header.frame_id を
    //   無視してこの値を使う。sim では "d435_depth_frame" を指定すること。
    source_frame_ = this->declare_parameter<std::string>("source_frame", "");

    // TF バッファ/リスナ (ノードのクロック = use_sim_time に追従)
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/converted_pointcloud", 10,
      std::bind(&PcToOctomapNode::pointcloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_out", 10);

    RCLCPP_INFO(this->get_logger(),
      "PcToOctomapNode started. target_frame=%s, res=%.3f, max_range=%.2f",
      target_frame_.c_str(), octree_.getResolution(), max_range_);
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 入力点群のフレーム (source_frame_ が指定されていれば frame_id を上書き)
    const std::string src_frame =
      source_frame_.empty() ? msg->header.frame_id : source_frame_;

    // src_frame → target_frame の TF を取得
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(
        target_frame_, src_frame, msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s <- %s 取得失敗: %s", target_frame_.c_str(),
        src_frame.c_str(), ex.what());
      return;
    }

    // TransformStamped → Eigen 同次変換 (cloud_frame 点 → target_frame 点)
    const Eigen::Isometry3d T = tf2::transformToEigen(tf_msg);

    octomap::Pointcloud octo_cloud;
    octo_cloud.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x, y = *iter_y, z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      // センサ基準(=入力フレーム)で採用範囲を制限してから変換
      if (std::abs(x) <= max_range_ && std::abs(y) <= max_range_ && std::abs(z) <= max_range_) {
        const Eigen::Vector3d p = T * Eigen::Vector3d(x, y, z);
        octo_cloud.push_back(
          static_cast<float>(p.x()),
          static_cast<float>(p.y()),
          static_cast<float>(p.z()));
      }
    }

    // sensor_origin も target_frame でのカメラ位置 (= T の並進成分) に変換
    const Eigen::Vector3d origin = T.translation();
    const octomap::point3d sensor_origin(
      static_cast<float>(origin.x()),
      static_cast<float>(origin.y()),
      static_cast<float>(origin.z()));

    octree_.insertPointCloud(octo_cloud, sensor_origin);

    // ROS メッセージへ (フレームは target_frame で出す)
    octomap_msgs::msg::Octomap map_msg;
    map_msg.header.frame_id = target_frame_;
    map_msg.header.stamp = msg->header.stamp;

    if (octomap_msgs::binaryMapToMsg(octree_, map_msg)) {
      pub_->publish(map_msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to serialize OctoMap.");
    }
  }

  octomap::OcTree octree_;
  std::string target_frame_;
  std::string source_frame_;
  double max_range_;
  double tf_timeout_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcToOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
