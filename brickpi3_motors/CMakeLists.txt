cmake_minimum_required(VERSION 3.16)
project(brickpi3_motors LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

if(NOT DEFINED ENV{BRICKPI3_ROOT_DIR})
	message(FATAL_ERROR "You must set BRICKPI3_ROOT_DIR")
endif()

include_directories("$ENV{BRICKPI3_ROOT_DIR}/Software/C")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  brickpi3_motors
  SHARED
  src/brickpi3_motors.cpp
)
target_compile_features(brickpi3_motors PUBLIC cxx_std_17)
target_include_directories(brickpi3_motors PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/brickpi3_motors>
)
ament_target_dependencies(
  brickpi3_motors PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_2_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface brickpi3_motors.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/brickpi3_motors
)
install(TARGETS brickpi3_motors
  EXPORT export_brickpi3_motors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_brickpi3_motors HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
