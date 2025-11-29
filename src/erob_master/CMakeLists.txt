cmake_minimum_required(VERSION 3.8)
project(erob_master)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 包含目录
include_directories(
  include
)


set(srv_files
  "srv/ConfigureMotor.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  # 如果有其他 .msg 或 .action 文件也加在这里
)

# 添加erob_canoepn_ros2可执行文件
add_executable(erob_canoepn_ros2 src/erob_canopen_ros2.cpp)
rosidl_target_interfaces(erob_canoepn_ros2 ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(erob_canoepn_ros2 rclcpp std_msgs sensor_msgs std_srvs geometry_msgs)

# 安装可执行文件
install(TARGETS
  erob_canoepn_ros2
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rclcpp std_msgs sensor_msgs std_srvs geometry_msgs)

# 必须的函数调用
ament_package() 
