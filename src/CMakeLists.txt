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
install_dynamic_graph_plugin(device)

#
# create the dynamic graph manager library
#
add_library(
  dynamic_graph_manager SHARED
  dynamic_graph_manager.cpp ros.cpp ros_python_interpreter_server.cpp
  ros_python_interpreter_client.cpp)
# Includes
target_include_directories(
  dynamic_graph_manager PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                               $<INSTALL_INTERFACE:include>)
# Dependencies
ament_target_dependencies(dynamic_graph_manager rclcpp)
ament_target_dependencies(dynamic_graph_manager std_srvs)
target_link_libraries(dynamic_graph_manager dynamic-graph::dynamic-graph)
target_link_libraries(dynamic_graph_manager
                      dynamic-graph-python::dynamic-graph-python)
target_link_libraries(dynamic_graph_manager yaml_utils::yaml_utils)
target_link_libraries(dynamic_graph_manager Eigen3::Eigen)
target_link_libraries(dynamic_graph_manager shared_memory::shared_memory)
target_link_libraries(dynamic_graph_manager real_time_tools::real_time_tools)
target_link_libraries(dynamic_graph_manager device)
rosidl_target_interfaces(dynamic_graph_manager ${PROJECT_NAME}_msg_srv
                         "rosidl_typesupport_cpp")
# Set the location of the device.so in the rpath.
set_target_properties(
  dynamic_graph_manager
  PROPERTIES
    INSTALL_RPATH
    "${CMAKE_INSTALL_RPATH}:${CMAKE_INSTALL_PREFIX}/lib/dynamic_graph_plugins")
# Export the target.
ament_export_interfaces(export_dynamic_graph_manager HAS_LIBRARY_TARGET)
# For the installation
list(APPEND all_targets dynamic_graph_manager)
list(APPEND all_target_exports export_dynamic_graph_manager)

#
# Export some default entities used for ROS<->DG communication
#
set(LIST_ROS_ENTITIES
    # ros_robot_state_publisher ros_robot_state_publisher_mt
    ros_publish
    # ros_subscribe ros_queued_subscribe ros_tf_listener ros_time
)

foreach(ROS_ENTITY ${LIST_ROS_ENTITIES})
  add_library(
    ${ROS_ENTITY} SHARED ros_entities/${ROS_ENTITY}.cpp
                         ros_entities/dg_ros_mapping.cpp
  )
  # Includes
  target_include_directories(
    ${ROS_ENTITY} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
  # Linked libraries
  ament_target_dependencies(${ROS_ENTITY} rclcpp)
  ament_target_dependencies(${ROS_ENTITY} geometry_msgs)
  ament_target_dependencies(${ROS_ENTITY} std_msgs)
  target_link_libraries(${ROS_ENTITY} dynamic_graph_manager)
  target_link_libraries(${ROS_ENTITY} dynamic-graph::dynamic-graph)
  target_link_libraries(${ROS_ENTITY}
                        dynamic-graph-python::dynamic-graph-python)
  rosidl_target_interfaces(${ROS_ENTITY} ${PROJECT_NAME}_msg_srv
                           "rosidl_typesupport_cpp")
  # Export/Install
  install_dynamic_graph_plugin(${ROS_ENTITY})
endforeach()

# Install the libraries
install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
