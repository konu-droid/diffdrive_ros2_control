cmake_minimum_required(VERSION 3.16)
project(diffdrive_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set dependencies
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
  diffdrive_ros2_control
  SHARED
  src/diffdrive_system.cpp
)
target_compile_features(diffdrive_ros2_control PUBLIC cxx_std_17)
target_include_directories(diffdrive_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/diffdrive_ros2_control>
)
ament_target_dependencies(
  diffdrive_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(diffdrive_ros2_control PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ROS2_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive_ros2_control.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/diffdrive_ros2_control
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/diffdrive_ros2_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diffdrive_ros2_control
)
install(TARGETS diffdrive_ros2_control
  EXPORT export_diffdrive_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_diffdrive_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
