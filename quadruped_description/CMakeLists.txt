cmake_minimum_required(VERSION 3.5)
project(quadruped_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  # ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
endif()

ament_package()