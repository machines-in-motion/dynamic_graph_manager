# Here we create a simple dgm for the purpose of documentation.
# It displays how to build a new DGM for a specific robot.

# create the executable
add_executable(demo_dynamic_graph_manager
  main.cpp
  simple_dgm.hpp
)
# link the dependecies to it
target_link_libraries(demo_dynamic_graph_manager
  dynamic_graph_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
pkg_config_use_dependency(demo_dynamic_graph_manager yaml-cpp)
pkg_config_use_dependency(demo_dynamic_graph_manager dynamic-graph)
pkg_config_use_dependency(demo_dynamic_graph_manager dynamic-graph-python)
# add some preprocessor variable
target_compile_definitions(demo_dynamic_graph_manager PUBLIC
  TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/tests/config/"
)
