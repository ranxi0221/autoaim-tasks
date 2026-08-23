#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <autoaim_msgs/msg/orienta.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <filesystem>
#include <list>
#include <map>
#include <mutex>
#include <string>
#include <Eigen/Geometry>
#include <fmt/core.h>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tools/img_tools.hpp"

using namespace std::chrono;

class BagReplayNode : public rclcpp::Node
{
public:
  explicit BagReplayNode(const std::string & config_path)
  : Node("bag_replay_node")
  {
    // M3: 载入 YOLO 检测器（OpenVINO, CPU），debug 模式开启可视化窗口
    try {
      detector_ = std::make_unique<auto_aim::YOLO>(config_path, true);
      RCLCPP_INFO(this->get_logger(), "YOLO detector loaded from %s", config_path.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YOLO detector: %s", e.what());
      throw;
    }

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

    RCLCPP_INFO(this->get_logger(), "===== M3 BagReplayNode ready =====");
    RCLCPP_INFO(this->get_logger(), "Listening: /image_raw + /imu/quaternion (order-based pairing)");
  }

  int frame_count() const { return frame_count_; }
  int image_count() const { return image_count_; }
  int quat_count() const { return quat_count_; }

  // M3: 分类统计（color/name/type），节点退出时打印
  const std::map<std::string, int> & classify_stats() const { return classify_stats_; }

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

      // M3: 送进 YOLO 检测器（debug=true 自动弹出 "detection" 可视化窗口）
      int armor_count = -1;
      std::list<auto_aim::Armor> armors;
      try {
        armors = detector_->detect(cv_img, frame_count_);
        armor_count = static_cast<int>(armors.size());
        // 分类统计
        for (const auto & armor : armors) {
          auto key = fmt::format(
            "{}/{}/{}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
            auto_aim::ARMOR_TYPES[armor.type]);
          classify_stats_[key]++;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "detect failed: %s", e.what());
      }

      // M3: 自己再画一版检测框（YOLO debug 窗口在无显示器环境不可见），
      // 每 30 帧存一张截图到 shots/ 作为验收证据
      if (frame_count_ % 30 == 0) {
        auto vis = cv_img.clone();
        for (const auto & armor : armors) {
          tools::draw_points(vis, armor.points, {0, 255, 0}, 3);
          auto info = fmt::format(
            "{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color],
            auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type]);
          tools::draw_text(vis, info, armor.center, {0, 255, 0});
        }
        auto shot_path =
          fmt::format("shots/m3_frame{:04d}_armors{}.jpg", frame_count_, armor_count);
        try {
          std::filesystem::create_directories("shots");
          cv::imwrite(shot_path, vis);
          RCLCPP_INFO(this->get_logger(), "saved shot: %s", shot_path.c_str());
        } catch (const std::exception & e) {
          RCLCPP_WARN(this->get_logger(), "save shot failed: %s", e.what());
        }
      }

      // M2 验证输出 + M3 检测结果
      RCLCPP_INFO(this->get_logger(),
        "PAIRED [frame %d] stamp=%u | q=(%.3f,%.3f,%.3f,%.3f) | img=%dx%d | dt=%.2fms | armors=%d",
        frame_count_,
        img_msg->header.stamp.nanosec,
        q.w(), q.x(), q.y(), q.z(),
        cv_img.cols, cv_img.rows,
        dt_ms,
        armor_count);

      // 每 60 帧刷一次显示窗口（imshow 是异步的，waitKey 让它真正渲染）
      if (frame_count_ % 60 == 0) {
        cv::waitKey(1);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<autoaim_msgs::msg::Orienta>::SharedPtr quat_sub_;

  std::unique_ptr<auto_aim::YOLO> detector_;

  sensor_msgs::msg::Image::SharedPtr pending_image_;
  autoaim_msgs::msg::Orienta::SharedPtr pending_quat_;
  std::mutex mutex_;

  int image_count_ = 0;
  int quat_count_ = 0;
  int frame_count_ = 0;
  int64_t last_pair_stamp_ = 0;
  std::map<std::string, int> classify_stats_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 用法: ros2 run sp_vision bag_replay_node <config_path>
  std::string config_path = "configs/standard3.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

  std::shared_ptr<BagReplayNode> node;
  try {
    node = std::make_shared<BagReplayNode>(config_path);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Node init failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);

  RCLCPP_INFO(rclcpp::get_logger("main"),
    "Done. image=%d, quat=%d, paired=%d",
    node->image_count(), node->quat_count(), node->frame_count());

  RCLCPP_INFO(rclcpp::get_logger("main"), "===== M3 classify stats =====");
  for (const auto & [key, cnt] : node->classify_stats()) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "  %s: %d", key.c_str(), cnt);
  }

  rclcpp::shutdown();
  return 0;
}
