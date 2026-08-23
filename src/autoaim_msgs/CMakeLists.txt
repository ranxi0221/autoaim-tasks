cmake_minimum_required(VERSION 3.8)
project(autoaim_msgs)

# 默认安装到 ROS2 标准路径
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到 rosidl 生成器
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Orienta.msg"
  "msg/TargetState.msg"
)

# 导出以便其他包依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()
