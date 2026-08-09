#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <autoaim_msgs/msg/orienta.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <mutex>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

using namespace std::chrono;

class BagReplayNode : public rclcpp::Node
{
public:
  BagReplayNode()
  : Node("bag_replay_node")
  {
    // 订阅图像
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        image_count_++;
        pending_image_ = msg;
        try_pair();
      });

    // 订阅四元数
    quat_sub_ = this->create_subscription<autoaim_msgs::msg::Orienta>(
      "/imu/quaternion", rclcpp::SensorDataQoS(),
      [this](const autoaim_msgs::msg::Orienta::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        quat_count_++;
        pending_quat_ = msg;
        try_pair();
      });

    RCLCPP_INFO(this->get_logger(), "===== M2 BagReplayNode ready =====");
    RCLCPP_INFO(this->get_logger(), "Listening: /image_raw + /imu/quaternion (order-based pairing)");
  }

  int frame_count() const { return frame_count_; }
  int image_count() const { return image_count_; }
  int quat_count() const { return quat_count_; }

private:
  // 同帧同序 = 收到一对就配对
  void try_pair()
  {
    if (pending_image_ && pending_quat_) {
      frame_count_++;

      auto img_msg = pending_image_;
      auto quat_msg = pending_quat_;
      pending_image_.reset();
      pending_quat_.reset();

      // ROS Image → cv::Mat
      cv::Mat cv_img;
      try {
        cv_img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
        return;
      }

      // Orienta → Eigen::Quaterniond
      Eigen::Quaterniond q(quat_msg->w, quat_msg->x, quat_msg->y, quat_msg->z);

      // header.stamp nsec → steady_clock::time_point
      auto t = steady_clock::time_point(nanoseconds(img_msg->header.stamp.nanosec));

      // 计算帧间隔 dt
      double dt_ms = 0.0;
      if (last_pair_stamp_ > 0) {
        dt_ms = (img_msg->header.stamp.nanosec - last_pair_stamp_) / 1e6;
      }
      last_pair_stamp_ = img_msg->header.stamp.nanosec;

      // M2 验证输出
      RCLCPP_INFO(this->get_logger(),
        "PAIRED [frame %d] stamp=%ld | q=(%.3f,%.3f,%.3f,%.3f) | img=%dx%d | dt=%.2fms",
        frame_count_,
        img_msg->header.stamp.nanosec,
        q.w(), q.x(), q.y(), q.z(),
        cv_img.cols, cv_img.rows,
        dt_ms);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<autoaim_msgs::msg::Orienta>::SharedPtr quat_sub_;

  sensor_msgs::msg::Image::SharedPtr pending_image_;
  autoaim_msgs::msg::Orienta::SharedPtr pending_quat_;
  std::mutex mutex_;

  int image_count_ = 0;
  int quat_count_ = 0;
  int frame_count_ = 0;
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
