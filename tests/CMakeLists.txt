##################################
# this macro define a unit tests #
##################################
macro(create_dgm_unittest test_name)

# create the executable
catkin_add_gtest(${test_name}_ut
  main.cpp
  test_${test_name}.cpp
)
# link the dependecies to it
target_link_libraries(${test_name}_ut
    ${test_name}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
pkg_config_use_dependency(${test_name}_ut yaml-cpp)
pkg_config_use_dependency(${test_name}_ut dynamic-graph)
pkg_config_use_dependency(${test_name}_ut dynamic-graph-python)
# add some preprocessor variable
target_compile_definitions(${test_name}_ut PUBLIC
  TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/tests/config/"
)

endmacro(create_dgm_unittest test_name)

################################
# this macro define a rostests #
################################
macro(create_dgm_rostest test_name)
  if(CATKIN_ENABLE_TESTING)
    # fin the ros test package
    find_package(rostest REQUIRED)
    # Create the rostest
    add_rostest_gtest(${test_name}_rostest
      test_${test_name}.test # launch file that will be called before starting the test
      test_${test_name}.cpp # source code of test that uses ros parameter server
      main.cpp )
    # link the dependecies to it
    target_link_libraries(${test_name}_rostest
        dynamic_graph_manager
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
    )
    pkg_config_use_dependency(${test_name}_rostest yaml-cpp)
    pkg_config_use_dependency(${test_name}_rostest dynamic-graph)
    pkg_config_use_dependency(${test_name}_rostest dynamic-graph-python)
    # add some preprocessor variable
    target_compile_definitions(${test_name}_rostest PUBLIC
      TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/tests/config/"
    )
  endif(CATKIN_ENABLE_TESTING)
endmacro(create_dgm_rostest test_name)

################
# Add unittest #
################
create_dgm_unittest(device)

###############
# Add rostest #
###############
create_dgm_rostest(dynamic_graph_manager)
create_dgm_rostest(ros_init)
create_dgm_rostest(ros_interpreter)