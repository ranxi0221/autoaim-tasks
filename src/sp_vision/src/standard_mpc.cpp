#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  auto last_status_log = std::chrono::steady_clock::now();

  std::atomic<bool> quit = false;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};

  // 开火门控参数（ITL 行为：指令角 vs 电控反馈角，容差内才开火；auto_fire=false 可退回纯 planner.fire）
  auto yaml = YAML::LoadFile(config_path);
  auto auto_fire = yaml["auto_fire"] ? yaml["auto_fire"].as<bool>() : true;
  auto first_tol = (yaml["first_tolerance"] ? yaml["first_tolerance"].as<double>() : 1.8) / 57.3;
  auto second_tol = (yaml["second_tolerance"] ? yaml["second_tolerance"].as<double>() : 1.3) / 57.3;
  auto judge_distance = yaml["judge_distance"] ? yaml["judge_distance"].as<double>() : 2.0;
  // 强制工作档（摆臂测试台下位机无档位切换，恒发 mode=0）：force_mode: "auto_aim" 时忽略电控档位
  auto force_mode = yaml["force_mode"] ? yaml["force_mode"].as<std::string>() : "";
  // 敌方颜色（与 tracker 构造逻辑一致），运行时可用 r/b 键切换
  auto enemy_color =
    (yaml["enemy_color"] && yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red
                                                                            : auto_aim::Color::blue;

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      if (!target_queue.empty() && mode == io::GimbalMode::AUTO_AIM) {
        auto target = target_queue.front();
        auto gs = gimbal.state();
        auto plan = planner.plan(target, gs.bullet_speed);

        bool fire = plan.fire;
        if (plan.control && auto_fire && target.has_value()) {
          auto x = target->ekf_x();
          auto dist = std::hypot(x[0], x[2]);
          auto tol = dist > judge_distance ? second_tol : first_tol;
          fire = fire && std::abs(plan.yaw - gs.yaw) < tol && std::abs(plan.pitch - gs.pitch) < tol;
        }

        if (plan.control)
          gimbal.send(
            true, fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
            plan.pitch_acc);
        else  // ITL 行为：无目标时 mode=0 回显当前反馈角
          gimbal.send(false, false, gs.yaw, 0, 0, gs.pitch, 0, 0);

        std::this_thread::sleep_for(10ms);
      } else
        std::this_thread::sleep_for(200ms);
    }
  });

  while (!exiter.exit()) {
    mode = gimbal.mode();

    // 摆臂测试台：下位机恒发 mode=0，force_mode 强制进入自瞄档（真车不配此键）
    if (force_mode == "auto_aim") mode = io::GimbalMode::AUTO_AIM;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode.load();
    }

    camera.read(img, t);
    auto q = gimbal.q(t);
    auto gs = gimbal.state();
    recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);

    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      auto armors = yolo.detect(img);
      auto targets = tracker.track(armors, t);

      // 每秒打印一次检测状态（摆臂调试：不依赖 GUI 窗口也能确认检测/跟踪是否工作）
      if (tools::delta_time(t, last_status_log) > 1.0) {
        std::string pos_str;
        if (!targets.empty()) {
          auto x = targets.front().ekf_x();
          pos_str = fmt::format(" pos=({:.2f},{:.2f},{:.2f})m", x[0], x[2], x[4]);
        }
        tools::logger()->info(
          "[AutoAim] color={} armors={} targets={} state={}{}", auto_aim::COLORS[enemy_color],
          armors.size(), targets.size(), tracker.state(), pos_str);
        last_status_log = t;
      }

      if (!targets.empty())
        target_queue.push(targets.front());
      else
        target_queue.push(std::nullopt);
    }

    /// 打符
    else if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
      buff_solver.set_R_gimbal2world(q);

      auto power_runes = buff_detector.detect(img);

      buff_solver.solve(power_runes);

      auto_aim::Plan buff_plan;
      if (mode.load() == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      }
      gimbal.send(
        buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
        buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);

    } else
      gimbal.send(false, false, gs.yaw, 0, 0, gs.pitch, 0, 0);  // ITL 行为：IDLE 回显当前反馈角

    // 泵送 GUI 事件：没有 waitKey，YOLO 的 detection 窗口只闪现第一帧不刷新。
    // 同时支持键盘切换敌方颜色（演示用）：点击 detection 窗口获得焦点后，r=红色敌方，b=蓝色敌方
    auto key = cv::waitKey(1);
    if (key == 'r' && enemy_color != auto_aim::Color::red) {
      enemy_color = auto_aim::Color::red;
      tracker.set_enemy_color(enemy_color);
      tools::logger()->info("[AutoAim] 敌方颜色切换为 red（按键 r=红 / b=蓝）");
    } else if (key == 'b' && enemy_color != auto_aim::Color::blue) {
      enemy_color = auto_aim::Color::blue;
      tracker.set_enemy_color(enemy_color);
      tools::logger()->info("[AutoAim] 敌方颜色切换为 blue（按键 r=红 / b=蓝）");
    }
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  auto gs_end = gimbal.state();
  gimbal.send(false, false, gs_end.yaw, 0, 0, gs_end.pitch, 0, 0);

  return 0;
}