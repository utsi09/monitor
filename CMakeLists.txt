cmake_minimum_required(VERSION 3.8)
project(monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

add_executable(monitor_node
  src/main.cpp
  src/update_system_state.cpp
  # src/lidar_check.cpp
  # src/camera_check.cpp
  # src/imu_check.cpp
  # src/gps_check.cpp
  # src/calculate_ftti.cpp
)

target_include_directories(monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(monitor_node
  behaviortree_cpp::behaviortree_cpp
)

ament_target_dependencies(monitor_node
  rclcpp
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

install(TARGETS monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_tree
  DESTINATION share/${PROJECT_NAME})

ament_package()
