#
# Python wrapper #
#
add_library(${PROJECT_NAME}_cpp_bindings MODULE
            dynamic_graph_manager.cpp ros_python_interpreter_client.cpp
            dgm_head.cpp)
# Link to the messages
target_link_libraries(${PROJECT_NAME}_cpp_bindings pybind11::module)
target_link_libraries(${PROJECT_NAME}_cpp_bindings ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_cpp_bindings ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_cpp_bindings
                      PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(
  ${PROJECT_NAME}_cpp_bindings
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
get_dynamic_graph_plugin_install_path(dg_plugin_install_path)
install(TARGETS ${PROJECT_NAME}_cpp_bindings DESTINATION ${PYTHON_INSTALL_DIR})
set_target_properties(
  ${PROJECT_NAME}_cpp_bindings
  PROPERTIES
    INSTALL_RPATH
    "${CMAKE_INSTALL_RPATH}:${CMAKE_INSTALL_PREFIX}/${dg_plugin_install_path}")
