cmake_minimum_required(VERSION 3.8)
project(traffic_light_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(traffic_light_pkg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(traffic_light_plugin SHARED src/traffic_light_plugin.cpp include/traffic_light_plugin.hpp)

ament_target_dependencies(traffic_light_plugin
  gazebo_ros
  rclcpp
  traffic_light_pkg
  std_msgs
)

target_link_libraries(traffic_light_plugin ${GAZEBO_LIBRARIES})

install(TARGETS traffic_light_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
