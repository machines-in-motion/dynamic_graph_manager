###################
## add unit tests #
###################

macro(create_dynamic_graph_manager_unittest test_name)

# create the executable
catkin_add_gtest(test_${test_name}
  main.cpp
  test_${test_name}.cpp
)
# link the dependecies to it
target_link_libraries(test_${test_name}
    ${test_name}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
pkg_config_use_dependency(test_${test_name} yaml-cpp)
pkg_config_use_dependency(test_${test_name} dynamic-graph)
pkg_config_use_dependency(test_${test_name} dynamic-graph-python)
# add some preprocessor variable
target_compile_definitions(test_${test_name} PUBLIC
  TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/tests/config/"
)

endmacro(create_dynamic_graph_manager_unittest test_name)

create_dynamic_graph_manager_unittest(dynamic_graph_manager)
create_dynamic_graph_manager_unittest(device)