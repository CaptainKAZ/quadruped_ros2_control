cmake_minimum_required(VERSION 3.5)
project(odrive_cansimple_hardware_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(
  odrive_cansimple_hardware_interface
  SHARED
  src/cansimple_hardware_interface.cpp
  src/socketcan.cpp
)
target_include_directories(
  odrive_cansimple_hardware_interface
  PUBLIC
  include
)
ament_target_dependencies(
  odrive_cansimple_hardware_interface
  hardware_interface
  rclcpp
)

# prevent pluginlib from using boost
target_compile_definitions(odrive_cansimple_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface odrive_cansimple_hardware_interface.xml)

install(
  TARGETS
  odrive_cansimple_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_cansimple_hardware_interface test/test_cansimple_hardware_interface.cpp)
  target_include_directories(test_cansimple_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_cansimple_hardware_interface
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
  add_executable(
    fake_resource_manager
    test/fake_resource_manager.cpp
  )
  target_link_libraries(
    fake_resource_manager
    odrive_cansimple_hardware_interface
  )
  add_executable(
    torque_control
    test/torque_control.cpp
  )
  target_link_libraries(
    torque_control
    odrive_cansimple_hardware_interface
  )
  add_executable(
    pos_control
    test/pos_control.cpp
  )
  target_link_libraries(
    pos_control
    odrive_cansimple_hardware_interface
  )
  add_executable(
    vel_control
    test/vel_control.cpp
  )
  target_link_libraries(
    vel_control
    odrive_cansimple_hardware_interface
  )
  install(
    TARGETS
    fake_resource_manager
    vel_control
    pos_control
    torque_control
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  odrive_cansimple_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
