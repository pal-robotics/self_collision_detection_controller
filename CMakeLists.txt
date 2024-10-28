cmake_minimum_required(VERSION 3.16)
project(safety_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  parameter_traits
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  joint_limits
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(safety_controller_parameters
  src/safety_controller.yaml
)

add_library(safety_controller SHARED
  src/safety_controller.cpp
)
target_compile_features(safety_controller PUBLIC cxx_std_17)
target_include_directories(safety_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/safety_controller>
)
target_link_libraries(safety_controller PUBLIC
  safety_controller_parameters
)
ament_target_dependencies(safety_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(safety_controller PRIVATE "safety_controller__VISIBILITY_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface safety_controller.xml)

if(BUILD_TESTING)
 
endif()

install(
  DIRECTORY include/
  DESTINATION include/safety_controller
)

install(TARGETS
    safety_controller
    safety_controller_parameters
  EXPORT export_safety_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_safety_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
