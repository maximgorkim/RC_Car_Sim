cmake_minimum_required(VERSION 3.8)

project(pedestrian_object_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(pedestrian_object_plugin SHARED
 src/geometric_point.cpp
 src/pedestrian_object_plugin.cpp 
 include/geometric_point.h
 include/pedestrian_object_plugin.h
)

ament_target_dependencies(pedestrian_object_plugin
  gazebo_ros
)

target_link_libraries(pedestrian_object_plugin ${GAZEBO_LIBRARIES})

install(TARGETS pedestrian_object_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
