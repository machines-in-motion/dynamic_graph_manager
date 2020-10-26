#
# this macro define a unit tests
#
macro(create_dgm_unittest test_name dependencies)

  # Set a general config folder path for all tests
  set(TEST_CONFIG_PATH ${PROJECT_SOURCE_DIR}/tests/config/)

  # Create the cmake target using ament and gtest.
  ament_add_gtest(test_${test_name} main.cpp test_${test_name}.cpp)

  # Include dependencies.
  target_include_directories(
    test_${test_name}
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/demos>
           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/tests>
           $<INSTALL_INTERFACE:include> SYSTEM
    PUBLIC ${PYTHON_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

  # Link the dependecies to it.
  target_link_libraries(test_${test_name} dynamic_graph_manager)

  # add some preprocessor variable
  target_compile_definitions(
    test_${test_name}
    PUBLIC TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/tests/config/")
endmacro(create_dgm_unittest test_name)

#
# Add unittest
#
if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  create_dgm_unittest(device device)
  create_dgm_unittest(cpp_features "")
  create_dgm_unittest(dgm_control dynamic_graph_manager)
  create_dgm_unittest(dgm_hardware dynamic_graph_manager)
  create_dgm_unittest(dgm_common dynamic_graph_manager)
  create_dgm_unittest(ros_init dynamic_graph_manager)
  create_dgm_unittest(ros_interpreter dynamic_graph_manager)
endif()
