cmake_minimum_required(VERSION 3.8)
project(self_collision_detection_controller)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

set(_Boost_PYTHON310_HEADERS "boost/python.hpp")
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
generate_parameter_library(collision_controller_parameters
  src/collision_controller.yaml
)

ament_auto_add_library(self_collision_detection_controller SHARED
  src/collision_controller.cpp
)
target_include_directories(self_collision_detection_controller PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  pinocchio::pinocchio
)

target_link_libraries(self_collision_detection_controller 
  collision_controller_parameters 
  ${EIGEN3_LIBRARIES}
  pinocchio::pinocchio
  
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.

pluginlib_export_plugin_description_file(controller_interface collision_controller.xml)

ament_auto_package(INSTALL_TO_SHARE launch config
USE_SCOPED_HEADER_INSTALL_DIR) 

