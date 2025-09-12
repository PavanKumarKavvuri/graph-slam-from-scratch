cmake_minimum_required(VERSION 3.8)
project(graph-slam-from-scratch)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(opteran_spatial_msgs REQUIRED)
find_package(opteran_vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

include_directories(include)

add_executable(graph_slam src/main.cpp src/graph_slam.cpp)
# ament_target_dependencies(graph_node rclcpp std_msgs)

target_link_libraries(graph_slam
  PRIVATE rclcpp::rclcpp
  PUBLIC ${nav_msgs_TARGETS}
  PUBLIC ${geometry_msgs_TARGETS}
  PUBLIC ${std_msgs_TARGETS})

install(TARGETS
  graph_slam
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
