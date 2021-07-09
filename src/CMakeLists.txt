set(all_src_targets)

if (BUILD_ROS_DYNAMIC_GRAPH)
  #
  # Create the device library
  #
  add_library(
    device SHARED
    device.cpp
    device_simulator.cpp
    periodic-call.cpp
    tools.cpp
    exception/exception-abstract.cpp
    exception/exception-factory.cpp
    exception/exception-signal.cpp
    exception/exception-tools.cpp
    exception/exception-dynamic.cpp
    exception/exception-feature.cpp
    exception/exception-task.cpp
    exception/exception-yaml-cpp.cpp)
  # Add the include dependencies.
  target_include_directories(
    device PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(device dynamic-graph::dynamic-graph)
  target_link_libraries(device dynamic-graph-python::dynamic-graph-python)
  target_link_libraries(device yaml_utils::yaml_utils)
  # Install the target and it's python bindings.
  install_dynamic_graph_plugin_python_bindings(device)
  # Install the plugin.
  get_dynamic_graph_plugin_install_path(plugin_install_path)
  install(
    TARGETS device
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION ${plugin_install_path}
    ARCHIVE DESTINATION ${plugin_install_path}
    RUNTIME DESTINATION ${plugin_install_path}
    INCLUDES
    DESTINATION include)

  #
  # create the dynamic graph manager library
  #
  add_library(
    ${PROJECT_NAME} SHARED
    dynamic_graph_manager.cpp dgm_head.cpp ros.cpp
    # The hardware process implementation.
    hardware_process.cpp
    ros_python_interpreter_server.cpp ros_python_interpreter_client.cpp)
  # Define flag for the hardware process.
  target_compile_definitions(${PROJECT_NAME}
      PUBLIC BUILD_WITH_ROS_DYNAMIC_GRAPH="true")

  # Includes
  target_include_directories(
    ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)
  # Dependencies
  ament_target_dependencies(${PROJECT_NAME} rclcpp)
  ament_target_dependencies(${PROJECT_NAME} std_srvs)
  ament_target_dependencies(${PROJECT_NAME} std_msgs)
  ament_target_dependencies(${PROJECT_NAME} geometry_msgs)
  ament_target_dependencies(${PROJECT_NAME} mim_msgs)
  target_link_libraries(${PROJECT_NAME} dynamic-graph::dynamic-graph)
  target_link_libraries(${PROJECT_NAME}
                        dynamic-graph-python::dynamic-graph-python)
  target_link_libraries(${PROJECT_NAME} yaml_utils::yaml_utils)
  target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} shared_memory::shared_memory)
  target_link_libraries(${PROJECT_NAME} real_time_tools::real_time_tools)
  target_link_libraries(${PROJECT_NAME} device)
  # Set the location of the device.so in the rpath.
  set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
      INSTALL_RPATH
      "${CMAKE_INSTALL_RPATH}:${CMAKE_INSTALL_PREFIX}/${plugin_install_path}")
  # Export the main library.
  list(APPEND all_src_targets ${PROJECT_NAME})

  #
  # Export some default entities used for ROS<->DG communication
  #
  # deprecated module: ros_queued_subscribe ros_tf_listener ros_time
  # ros_robot_state_publisher ros_robot_state_publisher_mt
  set(ROS_ENTITIES_SRC
      ros_entities/ros_publish.cpp ros_entities/ros_subscribe.cpp
      ros_entities/dg_ros_mapping.cpp)

  add_library(ros_entities SHARED ${ROS_ENTITIES_SRC})
  # Includes
  target_include_directories(
    ros_entities PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)
  # Linked libraries
  target_link_libraries(ros_entities dynamic_graph_manager)
  # Install the target and it's python bindings.
  install_dynamic_graph_plugin_python_bindings(ros_entities)
  # Install the plugin.
  get_dynamic_graph_plugin_install_path(plugin_install_path)
  install(
    TARGETS ros_entities
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION ${plugin_install_path}
    ARCHIVE DESTINATION ${plugin_install_path}
    RUNTIME DESTINATION ${plugin_install_path}
    INCLUDES
    DESTINATION include)
else()
  #
  # create the dynamic graph manager library
  #
  add_library(
    ${PROJECT_NAME} SHARED
    # The hardware process implementation.
    hardware_process.cpp
    # The DG head code implementation.
    dgm_head.cpp
    )
  # Includes
  target_include_directories(
    ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)
  # Dependencies
  target_link_libraries(${PROJECT_NAME} yaml_utils::yaml_utils)
  target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} shared_memory::shared_memory)
  target_link_libraries(${PROJECT_NAME} real_time_tools::real_time_tools)
  # Export the main library.
  list(APPEND all_src_targets ${PROJECT_NAME})
endif()

# Install the libraries
install(
  TARGETS ${all_src_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

#
# Delete the ShM program
#
add_executable(delete_dgm_shm "program/delete_dgm_shm.cpp")
# Add the include dependencies.
target_include_directories(
  delete_dgm_shm PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(delete_dgm_shm shared_memory::shared_memory)
# install so that ros2 run sees it.
install(
  TARGETS delete_dgm_shm
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)
