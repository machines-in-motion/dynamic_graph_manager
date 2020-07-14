################################
# python bindings with pybind11#
################################
pybind11_add_module(py_${PROJECT_NAME}
    dynamic_graph_manager.cpp
    ros_python_interpreter_client.cpp
)
target_link_libraries(
    py_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(py_${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  OUTPUT_NAME wrapper
)
add_dependencies(py_${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
install(TARGETS py_${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})