#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(dynamic_graph_manager)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies
#
set(Python_ADDITIONAL_VERSIONS 3.6)

# depend on ament macros
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS2 dependencies
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
# Declare the ROS message and service files to be built. This is here because
# otherwise the Python include_dir is not found...
rosidl_generate_interfaces(
  ${PROJECT_NAME}_msg_srv "msg/Matrix.msg" "msg/Vector.msg"
  "srv/RunPythonCommand.srv" "srv/RunPythonFile.srv" "srv/TestUserCmdBool.srv")

# Standard dependencies.
find_package(Eigen3 REQUIRED)
# Pybind11
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
# MiM dependencies.
find_package(mpi_cmake_modules REQUIRED)
find_package(Python REQUIRED)
find_package(shared_memory REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(yaml_utils REQUIRED)
# Dynamic graph dependencies.
find_package(dynamic-graph REQUIRED)
find_package(dynamic-graph-python REQUIRED)

# export de dependencies
ament_export_dependencies(
  std_srvs
  dynamic-graph
  dynamic-graph-python
  Eigen3
  shared_memory
  real_time_tools
  yaml_utils
  rosidl_default_runtime)

# prepare to export all needed targets
set(all_targets)
set(all_target_exports)

#
# manage python file install
#
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

#
# Install the ROS nodes
#
install(
  FILES ${PROJECT_SOURCE_DIR}/ros_nodes/remote_python_client.py
  RENAME remote_python_client
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

#
# Prepare the target installation
#
set(all_targets)
set(all_target_exports)

#
# manage the creation of the libraries and executables
#
add_subdirectory(src)

#
# manage python bindings
#
add_subdirectory(srcpy)

#
# manage the demos
#
add_subdirectory(demos)

#
# manage the unit tests
#
# add_subdirectory(tests) Add Python unit-tests catkin_add_nosetests(tests)

#
# Install the package
#
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package
#
ament_package()
