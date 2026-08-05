cmake_minimum_required(VERSION 3.8)
project(example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(utils REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/control.py
  src/camera.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
