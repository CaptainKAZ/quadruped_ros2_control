cmake_minimum_required(VERSION 3.5)
project(jpos_initializer_controller)

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
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

find_package(tinysplinecxx REQUIRED)
# rename the following to let ament cmake find it
set(tinysplinecxx_LIBRARY_DIRS ${TINYSPLINECXX_LIBRARY_DIRS})
set(tinysplinecxx_INCLUDE_DIRS ${TINYSPLINECXX_INCLUDE_DIRS})
set(tinysplinecxx_LIBRARIES ${TINYSPLINECXX_LIBRARIES})
set(tinysplinecxx_DEFINITIONS ${TINYSPLINECXX_DEFINITIONS})

add_library(
  jpos_initializer
  SHARED
  src/jpos_initializer.cpp
)

target_include_directories(
  jpos_initializer
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  jpos_initializer
  tinysplinecxx
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

# prevent pluginlib from using boost
# target_compile_definitions(jpos_initializer PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  controller_interface jpos_initializer_controller.xml)

install(
  TARGETS
  jpos_initializer
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES jpos_initializer_controller.xml
  DESTINATION share/${PROJECT_NAME}
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

  ament_add_gmock(test_load_jpos_initializer test/test_load_jpos_initializer.cpp)
  target_include_directories(test_load_jpos_initializer PRIVATE include)
  ament_target_dependencies(
    test_load_jpos_initializer
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  # ament_add_gmock(test_jpos_initializer test/test_jpos_initializer.cpp)
  # target_include_directories(test_jpos_initializer PRIVATE include)
  # target_link_libraries(test_jpos_initializer jpos_initializer)
  # ament_target_dependencies(
  #   test_jpos_initializer
  #   controller_interface
  #   hardware_interface
  # )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  jpos_initializer
)
ament_export_targets(
  export_${PROJECT_NAME}
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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
