cmake_minimum_required(VERSION 3.8)
project(car_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(car_plugin SHARED src/carlikerobot_ros_plugin.cpp src/carlikerobot.cpp)

ament_target_dependencies(car_plugin
  gazebo_ros
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

target_link_libraries(car_plugin ${GAZEBO_LIBRARIES})

install(TARGETS car_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

