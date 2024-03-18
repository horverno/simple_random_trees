cmake_minimum_required(VERSION 3.8)
project(simple_random_trees)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(INCLUDE_DIRS 
  include 
)
include_directories(${INCLUDE_DIRS})

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

add_executable(display_tree_node src/display_tree.cpp)
ament_target_dependencies(display_tree_node ${ament_dependencies} )

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
display_tree_node
DESTINATION lib/${PROJECT_NAME})

ament_package()