#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <autoaim_msgs/msg/orienta.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <chrono>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

using namespace std::chrono;

class BagReplayNode : public rclcpp::Node
{
public:
  BagReplayNode()
  : Node("bag_replay_node")
  {
    // 单独订阅两个 topic，各自计数
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        image_count_++;
        last_image_stamp_ = msg->header.stamp.nanosec;
      });

    quat_sub_ = this->create_subscription<autoaim_msgs::msg::Orienta>(
      "/imu/quaternion", 10,
      [this](const autoaim_msgs::msg::Orienta::SharedPtr msg) {
        quat_count_++;
      });

    // message_filters 近似时间戳配对
    sync_image_sub_.subscribe(this, "/image_raw");
    sync_quat_sub_.subscribe(this, "/imu/quaternion");

    sync_ = std::make_shared<Sync>(ImageQuatPolicy(10), sync_image_sub_, sync_quat_sub_);
    sync_->registerCallback(&BagReplayNode::paired_callback, this);

    RCLCPP_INFO(this->get_logger(), "===== M2 BagReplayNode ready =====");
    RCLCPP_INFO(this->get_logger(), "Listening: /image_raw + /imu/quaternion (ApproxTime sync)");
  }

  int frame_count() const { return frame_count_; }
  int image_count() const { return image_count_; }
  int quat_count() const { return quat_count_; }

private:
  void paired_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const autoaim_msgs::msg::Orienta::ConstSharedPtr & quat)
  {
    frame_count_++;

    // 转换图像
    cv::Mat cv_img;
    try {
      cv_img = cv_bridge::toCvCopy(img, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    // 提取四元数
    Eigen::Quaterniond q(quat->w, quat->x, quat->y, quat->z);

    // 时间戳 → steady_clock::time_point
    auto t = steady_clock::time_point(nanoseconds(img->header.stamp.nanosec));

    // 计算帧间隔
    double dt_ms = 0.0;
    if (last_pair_stamp_ > 0) {
      dt_ms = (img->header.stamp.nanosec - last_pair_stamp_) / 1e6;
    }
    last_pair_stamp_ = img->header.stamp.nanosec;

    // M2 验证: 配对成功
    RCLCPP_INFO(this->get_logger(),
      "PAIRED [frame %d] stamp=%ld | q=(%.3f,%.3f,%.3f,%.3f) | img=%dx%d | dt=%.2fms",
      frame_count_,
      img->header.stamp.nanosec,
      q.w(), q.x(), q.y(), q.z(),
      cv_img.cols, cv_img.rows,
      dt_ms);
  }

  // 独立订阅
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<autoaim_msgs::msg::Orienta>::SharedPtr quat_sub_;
  int image_count_ = 0;
  int quat_count_ = 0;

  // 同步配对
  message_filters::Subscriber<sensor_msgs::msg::Image> sync_image_sub_;
  message_filters::Subscriber<autoaim_msgs::msg::Orienta> sync_quat_sub_;
  using ImageQuatPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, autoaim_msgs::msg::Orienta>;
  using Sync = message_filters::Synchronizer<ImageQuatPolicy>;
  std::shared_ptr<Sync> sync_;

  int frame_count_ = 0;
  int64_t last_image_stamp_ = 0;
  int64_t last_pair_stamp_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BagReplayNode>();
  rclcpp::spin(node);

  RCLCPP_INFO(rclcpp::get_logger("main"),
    "Done. image=%d, quat=%d, paired=%d",
    node->image_count(), node->quat_count(), node->frame_count());

  rclcpp::shutdown();
  return 0;
}
