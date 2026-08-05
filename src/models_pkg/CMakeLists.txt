cmake_minimum_required(VERSION 3.8)
project(models_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install every model subdirectory (model.sdf, model.config, meshes, ...)
# into share/models_pkg/<model_name> so gazebo/spawn_entity can find them
# at install time. Without this, only package metadata gets installed and
# spawn_entity.py fails with "specified file ... does not exist".
file(GLOB model_dirs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(dir ${model_dirs})
  if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}/
    )
  endif()
endforeach()

ament_package()
