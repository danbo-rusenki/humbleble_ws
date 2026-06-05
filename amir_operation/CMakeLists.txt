cmake_minimum_required(VERSION 3.8)
project(amir_operation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(behavior_tree_msgs REQUIRED)

include_directories(
  include
)

add_executable(jointfix
  src/jointfix_pick_place.cpp
)

add_executable(jointfix10
  src/jointfix_10_pick_place.cpp
)

add_executable(pick_server
  src/pick_server.cpp
)

add_executable(initial_posi_gz
  src/initial_posi_gz.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
  behavior_tree_msgs
)

ament_target_dependencies(jointfix ${dependencies})
ament_target_dependencies(jointfix10 ${dependencies})
ament_target_dependencies(pick_server ${dependencies})
ament_target_dependencies(initial_posi_gz ${dependencies})

install(TARGETS
  jointfix
  jointfix10
  pick_server
  initial_posi_gz
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
