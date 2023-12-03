cmake_minimum_required(VERSION 3.8)
project(tb3_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################################################
# find dependencies
######################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED )
find_package(rclcpp_action REQUIRED )
find_package(behaviortree_cpp_v3 REQUIRED )
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

######################################################
set(dependencies
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp_v3)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(tb3_behavior_node src/tb3_behavior_node.cpp)
ament_target_dependencies(tb3_behavior_node
  std_msgs
  geometry_msgs 
  sensor_msgs 
  nav2_msgs
  OpenCV
  cv_bridge
  tf2_ros 
  tf2_geometry_msgs
  ${dependencies}
)

######################################################
install(TARGETS
  tb3_behavior_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

######################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
