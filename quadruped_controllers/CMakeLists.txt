cmake_minimum_required(VERSION 3.5)
project(quadruped_controllers)

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
find_package(control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(qpoases_vendor REQUIRED)

# temporary workaound to fix qpoases_vender bug
set (qpoases_vendor_LIBRARIES /opt/ros/foxy/lib/libqpOASES.a)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

set(FLAGS
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-maybe-uninitialized
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
        )

add_library(
  controller_base
  SHARED
  src/controller_base.cpp src/state_update.cpp src/pinocchio_solver.cpp src/ros2_node_interface.cpp
)
target_include_directories(
  controller_base
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  controller_base
  nav_msgs
  geometry_msgs
  pinocchio
  tf2
  tf2_msgs
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

add_library(swing_stance_controller
  SHARED
  src/swing_stance_controller.cpp
)

target_include_directories(
  swing_stance_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  swing_stance_controller
  nav_msgs
  geometry_msgs
  pinocchio
  tf2
  tf2_msgs
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

# standup controller
add_library(stand_up_controller
  SHARED
  src/stand_up_controller.cpp src/controller_base.cpp src/state_update.cpp src/pinocchio_solver.cpp src/ros2_node_interface.cpp
)

target_include_directories(
  stand_up_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  stand_up_controller
  nav_msgs
  geometry_msgs
  pinocchio
  tf2
  tf2_msgs
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)



# prevent pluginlib from using boost
target_compile_definitions(controller_base PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  controller_interface controller_base.xml)
pluginlib_export_plugin_description_file(
    controller_interface swing_stance_controller.xml)

pluginlib_export_plugin_description_file(
    controller_interface stand_up_controller.xml)

install(
  TARGETS
  controller_base
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS
  stand_up_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS
  swing_stance_controller
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
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_controller_base test/test_load_controller_base.cpp)
  target_include_directories(test_load_controller_base PRIVATE include)
  ament_target_dependencies(
    test_load_controller_base
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  # ament_add_gmock(test_controller_base test/test_controller_base.cpp)
  # target_include_directories(test_controller_base PRIVATE include)
  # target_link_libraries(test_controller_base controller_base)
  # ament_target_dependencies(
  #   test_controller_base
  #   controller_interface
  #   hardware_interface
  # )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  controller_base
  swing_stance_controller
  stand_up_controller
)
ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

target_compile_options(controller_base PUBLIC ${FLAGS})
target_compile_options(stand_up_controller PUBLIC ${FLAGS})

ament_package()
