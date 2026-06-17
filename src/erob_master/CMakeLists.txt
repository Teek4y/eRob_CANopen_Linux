cmake_minimum_required(VERSION 3.8)
project(erob_master)

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

# 自动收集 msg/ 与 srv/ 目录下的所有接口文件
file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  # 如果有其他 .msg 或 .action 文件也加在这里
)

# 添加erob_canoepn_ros2可执行文件
add_executable(erob_canopen_ros2 src/erob_canopen_ros2.cpp)

# 
rosidl_get_typesupport_target(
  typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(erob_canopen_ros2 
  ${typesupport_target}
)

ament_target_dependencies(erob_canopen_ros2 rclcpp std_msgs sensor_msgs std_srvs geometry_msgs)

# 安装可执行文件
install(TARGETS
  erob_canopen_ros2
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
# ament_export_dependencies(rclcpp std_msgs sensor_msgs std_srvs geometry_msgs)

# Python package install (setup.py will handle console entry points)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 防止重复创建 ament_cmake_python 的自定义目标（例如在某些 CMake 配置中会重复加载）
if(NOT TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()


install(PROGRAMS scripts/joint_state_bridge DESTINATION lib/${PROJECT_NAME})
# Note: the `scripts/joint_state_bridge` is provided via the package's
# `setup.py` entry points. Installing it again with `install(PROGRAMS ...)`
# would create duplicate CMake targets, so we don't install it here.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 必须的函数调用
ament_package() 
