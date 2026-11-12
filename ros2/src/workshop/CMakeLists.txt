cmake_minimum_required(VERSION 3.8)
project(workshop_offboard_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(workshop_offboard_node
  src/workshop_offboard_node.cpp
)

ament_target_dependencies(workshop_offboard_node
  rclcpp
  px4_msgs
  geometry_msgs
)

install(TARGETS
  workshop_offboard_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()