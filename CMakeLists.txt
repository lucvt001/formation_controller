cmake_minimum_required(VERSION 3.8)
project(formation_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(tuper_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

set(ros_dependencies
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  nav_msgs
  tuper_interfaces
  tf2
  tf2_ros
)

add_executable(pid_server src/pid.cpp src/pid_server.cpp)
ament_target_dependencies(pid_server ${ros_dependencies})

add_executable(tf_repub src/tf_repub.cpp)
ament_target_dependencies(tf_repub ${ros_dependencies})

add_executable(gps_heading_to_tf src/gps_heading_to_tf.cpp)
ament_target_dependencies(gps_heading_to_tf ${ros_dependencies} geodesy)

add_executable(get_differential_value src/get_differential_value.cpp)
ament_target_dependencies(get_differential_value ${ros_dependencies})

add_executable(sum_and_scale src/sum_and_scale.cpp)
ament_target_dependencies(sum_and_scale ${ros_dependencies})

add_executable(origin_pub src/origin_pub.cpp)
ament_target_dependencies(origin_pub ${ros_dependencies})

add_executable(formation_shape_broadcaster src/formation_shape_broadcaster.cpp)
ament_target_dependencies(formation_shape_broadcaster ${ros_dependencies} yaml-cpp)
target_link_libraries(formation_shape_broadcaster ${YAML_CPP_LIBRARIES})

add_executable(tf_to_path src/tf_to_path.cpp)
ament_target_dependencies(tf_to_path ${ros_dependencies})

install(TARGETS
  pid_server
  tf_repub
  gps_heading_to_tf
  get_differential_value
  sum_and_scale
  origin_pub
  formation_shape_broadcaster
  tf_to_path
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include/
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
