cmake_minimum_required(VERSION 3.8)
project(bno055_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(bno055_plugin SHARED src/bno055_plugin.cpp include/bno055_plugin.hpp)

ament_target_dependencies(bno055_plugin
  gazebo_ros
  rclcpp
  sensor_msgs
)

target_link_libraries(bno055_plugin ${GAZEBO_LIBRARIES})

install(TARGETS bno055_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
