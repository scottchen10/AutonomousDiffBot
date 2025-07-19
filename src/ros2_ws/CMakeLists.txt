cmake_minimum_required(VERSION 3.8)
project(autonomous_diff_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(controller_manager 4.0.0)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  differential_drive_interface
  SHARED
  hardware/differential_drive_interface.cpp
)
target_compile_features(differential_drive_interface PUBLIC cxx_std_17)
target_include_directories(differential_drive_interface PUBLIC
target_compile_features(differential_drive_interface PUBLIC cxx_std_17)
target_include_directories(differential_drive_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/differential_drive_interface>
/usr/include
)
ament_target_dependencies(
  differential_drive_interface PUBLIC
  differential_drive_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(differential_drive_interface
  PUBLIC
  serial
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)

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

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/serial)
target_link_libraries(differential_drive_interface serial)

install(
  DIRECTORY hardware/include/
  DESTINATION include/autonomous_diff_bot
)

install(TARGETS differential_drive_interface
  EXPORT export_differential_drive_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS differential_drive_interface
  EXPORT export_differential_drive_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY launch description hardware
  DIRECTORY launch description hardware
  DESTINATION share/${PROJECT_NAME}
)
ament_export_targets(export_differential_drive_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_differential_drive_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()