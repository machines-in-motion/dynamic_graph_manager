# Here we create a simple dgm for the purpose of documentation. It displays how
# to build a new DGM for a specific robot.

# create the executable
add_executable(demo_dynamic_graph_manager main.cpp simple_dgm.hpp)
# link the dependecies to it
target_link_libraries(demo_dynamic_graph_manager dynamic_graph_manager)
rosidl_target_interfaces(demo_dynamic_graph_manager ${PROJECT_NAME}_msg_srv
                         "rosidl_typesupport_cpp")
# Set the location of the device.so in the rpath.
set_target_properties(
  demo_dynamic_graph_manager
  PROPERTIES
    INSTALL_RPATH
    "${CMAKE_INSTALL_RPATH}:${CMAKE_INSTALL_PREFIX}/lib/dynamic_graph_plugins")
# add some preprocessor variable
install(FILES simple_robot.yaml DESTINATION share/resources)
target_compile_definitions(
  demo_dynamic_graph_manager
  PUBLIC DEMO_RESOURCES_PATH="${CMAKE_INSTALL_PREFIX}/share/resources/")
# install so that ros2 run sees it.
install(
  TARGETS demo_dynamic_graph_manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)
