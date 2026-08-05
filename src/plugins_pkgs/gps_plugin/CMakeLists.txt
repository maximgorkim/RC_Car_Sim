cmake_minimum_required(VERSION 3.8)
project(gps_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(gps_plugin SHARED src/gps_plugin.cpp include/gps_plugin.hpp)

ament_target_dependencies(gps_plugin
  gazebo_ros
  rclcpp
  nav_msgs
)

target_link_libraries(gps_plugin ${GAZEBO_LIBRARIES})

install(TARGETS gps_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
