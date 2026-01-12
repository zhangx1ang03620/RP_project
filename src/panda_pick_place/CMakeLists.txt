cmake_minimum_required(VERSION 3.8)
project(panda_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# 1. 查找依赖
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)

# ------------------------------------------------------------------------------
# 2. 可执行文件
# ------------------------------------------------------------------------------
add_executable(pick_place_node src/mtc_tutorial.cpp)
ament_target_dependencies(mtc_tutorial
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
)

target_include_directories(mtc_tutorial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)

# ------------------------------------------------------------------------------
# 3. 安装规则
# ------------------------------------------------------------------------------
install(TARGETS mtc_tutorial
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# ------------------------------------------------------------------------------
# 4. lint 测试（可选）
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
