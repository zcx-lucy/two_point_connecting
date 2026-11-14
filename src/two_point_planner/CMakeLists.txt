cmake_minimum_required(VERSION 3.8)
project(two_point_planner)

# C++17：std::optional / 结构化绑定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(astar_planner src/astar_planner_node.cpp)
ament_target_dependencies(astar_planner
  rclcpp nav_msgs geometry_msgs visualization_msgs)

# 可执行文件装到 lib/<包名>（ros2 run 的约定位置）
install(TARGETS astar_planner DESTINATION lib/${PROJECT_NAME})
# launch / rviz 配置 / 地图原样装进 share/<包名>
install(DIRECTORY launch rviz maps DESTINATION share/${PROJECT_NAME})

ament_package()
