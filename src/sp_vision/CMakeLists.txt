cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# ament_cmake (ROS2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(autoaim_msgs REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs videoio calib3d)
# 兼容旧式 ${OpenCV_LIBS} 变量（OpenCV4 不再自动设置）
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_videoio opencv_calib3d opencv_dnn)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# 修复 fmt 包 INTERFACE_LINK_LIBRARIES 自带的 --as-needed 破坏链接顺序的问题
get_target_property(_fmt_iface_libs fmt::fmt INTERFACE_LINK_LIBRARIES)
if(_fmt_iface_libs)
  string(REPLACE "-Wl,--as-needed" "" _fmt_fixed "${_fmt_iface_libs}")
  set_target_properties(fmt::fmt PROPERTIES INTERFACE_LINK_LIBRARIES "${_fmt_fixed}")
endif()

# OpenVINO — Intel APT 版 (ABI=1，与系统库一致)
find_package(OpenVINO QUIET)
if(OpenVINO_FOUND)
  message(STATUS "OpenVINO found: ${OpenVINO_DIR} (ABI=${OpenVINO_GLIBCXX_USE_CXX11_ABI})")
else()
  message(WARNING "OpenVINO not found! NN-based detection (YOLO) will be disabled.")
endif()

find_package(Ceres QUIET)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
if(OpenVINO_FOUND)
  add_subdirectory(tasks/auto_aim)
  add_subdirectory(tasks/omniperception)
  if(Ceres_FOUND)
    add_subdirectory(tasks/auto_buff)
  endif()
endif()

####################src###################
if(OpenVINO_FOUND)
add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)

target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

	#################### ROS2 bag replay node (M2/M3) ####################
	add_executable(bag_replay_node src/bag_replay_node.cpp)
	target_link_libraries(bag_replay_node ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io Eigen3::Eigen)
	ament_target_dependencies(bag_replay_node
	  rclcpp sensor_msgs autoaim_msgs cv_bridge)
endif()

	##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(capture_gimbal calibration/capture_gimbal.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(capture_gimbal ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################tests##################
# 不需要 OpenVINO 的测试
add_executable(camera_test tests/camera_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(gimbal_frame_test tests/gimbal_frame_test.cpp)

target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(gimbal_frame_test io)

# 需要 OpenVINO (auto_aim/auto_buff) 的测试
if(OpenVINO_FOUND)
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)
add_executable(gimbal_test tests/gimbal_test.cpp)
add_executable(planner_test tests/planner_test.cpp)
add_executable(planner_test_offline tests/planner_test_offline.cpp)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
endif()

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
# 只有在找到 sp_msgs + OpenVINO 时才会添加 sentry
if(sp_msgs_FOUND AND OpenVINO_FOUND)
    # 编译 sentry 和 publish_test
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()

# 安装所有可执行文件到 ROS2 标准路径
install(TARGETS
  # M2 ROS2 bag replay node
  bag_replay_node
  # src executables (auto_aim + auto_buff)
  standard mt_standard standard_mpc
  auto_aim_debug_mpc mt_auto_aim_debug
  auto_buff_debug auto_buff_debug_mpc
  uav uav_debug
  # calibration
  capture capture_gimbal calibrate_camera calibrate_handeye
  calibrate_robotworld_handeye split_video
  # tests (no OpenVINO)
  camera_test cboard_test fire_test gimbal_response_test
  multi_usbcamera_test usbcamera_test dm_test
  gimbal_frame_test
  # tests (need OpenVINO)
  auto_aim_test auto_buff_test camera_detect_test
  camera_thread_test detector_video_test usbcamera_detect_test
  handeye_test minimum_vision_system gimbal_test
  planner_test planner_test_offline
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
