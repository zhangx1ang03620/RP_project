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

add_executable(pick_place_node src/mtc.cpp)

# 给 pick_place_node 添加依赖
ament_target_dependencies(pick_place_node
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
)

# 设置 C++ 标准
target_compile_features(pick_place_node PUBLIC c_std_99 cxx_std_17)



# ------------------------------------------------------------------------------
# 3. 安装规则
# ------------------------------------------------------------------------------
# 安装可执行文件
install(TARGETS pick_place_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# ------------------------------------------------------------------------------
# 4. lint 测试
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
