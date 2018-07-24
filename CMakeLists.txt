##################################################################################
# Add custom (non compiling) targets so non cpp files show up in QT_CREATOR IDE. #
##################################################################################
set(USE_QTCREATOR_${PROJECT_NAME} OFF CACHE BOOL
    "Set to ON if you want QtCreator to display all files in the project tree")
if(USE_QTCREATOR_${PROJECT_NAME})
    file(GLOB_RECURSE EXTRA_FILES
        *.h *.hh *.hpp *.py
        *.cf  *.yaml *.txt *.xml
        start *.launch *.urdf CATKIN_IGNORE *.in
    )
    list(FILTER EXTRA_FILES EXCLUDE REGEX "build")
    add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY
        ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
endif()

######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(dynamic_graph_manager)

# required to use std::shared_ptr in code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################
find_package(PkgConfig REQUIRED)
set(CATKIN_PKGS ${CATKIN_PKGS}
    roscpp
    dynamic_graph
    dynamic_graph_python
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

#add_required_dependency("roscpp")
##add_required_dependency("pinocchio >= 1.2.9")
#add_required_dependency("dynamic-graph >= 3.0.0")
#add_required_dependency("dynamic-graph-python")
##add_required_dependency("sot-core")
##add_required_dependency("sot-dynamic-pinocchio >= 3.0.0")
##add_required_dependency("dynamic_graph_bridge_msgs")
##add_required_dependency("dynamic_graph_bridge >= 3.0.0")
#SET(BOOST_COMPONENTS filesystem system thread)
#SEARCH_FOR_BOOST()
#SEARCH_FOR_EIGEN()
#FINDPYTHON()


######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
#    ${PYTHON_INCLUDE_PATH}
    ${PROJECT_SOURCE_DIR}/include
#    ${Eigen_INCLUDE_DIRS}
)

###########################################
## export the package as a catkin package #
###########################################
#catkin_package(
# INCLUDE_DIRS include
# CATKIN_DEPENDS ${CATKIN_PKGS}
#)

############################
## define the source files #
############################

#set(MAIN_SOURCE_FILES
#    src/sot_loader_light.cpp
#    src/main.cpp
#)

##############################
## create the device library #
##############################

#SET(DEVICE_NAME dg-simple-robot-device)
#ADD_LIBRARY(${DEVICE_NAME} SHARED
#  src/abstract_sot_external_interface/dg_simple_robot_device.cpp
#)
## Link the dynamic library containing the SoT with its dependencies.
#PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph")
#PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph-python")
#PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic_graph_bridge")
#PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "sot-core")
#IF(UNIX AND NOT APPLE)
#  TARGET_LINK_LIBRARIES(${DEVICE_NAME} ${Boost_LIBRARIES})
#ENDIF(UNIX AND NOT APPLE)


## build python submodule
#STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
#set(PYTHON_DG_MODULE_DIR_LOCAL
#    python2.7/dist-packages/dynamic_graph/sot/${PROJECT_NAME})
#set(PYTHON_DG_MODULE_DIR
#    ${PROJECT_BINARY_DIR}/devel/lib/${PYTHON_DG_MODULE_DIR_LOCAL})

#CMAKE_POLICY(PUSH)
#IF(POLICY CMP0037)
#  CMAKE_POLICY(SET CMP0037 OLD)
#ENDIF()
#ADD_LIBRARY(sot-simple-robot-${PYTHON_LIBRARY_NAME}-wrap
#  MODULE
#  ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/python-module-py.cc)
#SET_TARGET_PROPERTIES(sot-simple-robot-${PYTHON_LIBRARY_NAME}-wrap
#  PROPERTIES PREFIX ""
#  OUTPUT_NAME ${PYTHON_DG_MODULE_DIR_LOCAL}/wrap
# )
#CMAKE_POLICY(POP)
#TARGET_LINK_LIBRARIES(sot-simple-robot-${PYTHON_LIBRARY_NAME}-wrap "-Wl,--no-as-needed")
#TARGET_LINK_LIBRARIES(sot-simple-robot-${PYTHON_LIBRARY_NAME}-wrap ${DEVICE_NAME} ${PYTHON_LIBRARY})

#FILE(MAKE_DIRECTORY ${PYTHON_DG_MODULE_DIR})
#CONFIGURE_FILE(
#  ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/submodule/__init__.py.cmake
#  ${PYTHON_DG_MODULE_DIR}/__init__.py
#  )

##################################
## create the controller library #
##################################

#SET(CONTROLLER_NAME dg-simple-robot-controller)
#ADD_LIBRARY(${CONTROLLER_NAME} SHARED
#  src/abstract_sot_external_interface/dg_simple_robot_controller.cpp
#)

## Link the dynamic library containing the SoT with its dependencies.
#PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph")
#PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph-python")
#PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic_graph_bridge")
#PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-core")
#ADD_DEPENDENCIES(${CONTROLLER_NAME} ${DEVICE_NAME})
#TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${DEVICE_NAME})
#TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "ros_bridge")
#TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "ros_interpreter")

#IF(UNIX AND NOT APPLE)
#  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${Boost_LIBRARIES})
#ENDIF(UNIX AND NOT APPLE)

##INSTALL(TARGETS ${CONTROLLER_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

###############################
## create the main executable #
###############################
#add_executable(run_dynamic_graph ${MAIN_SOURCE_FILES})
## link the dependencies
#target_link_libraries(run_dynamic_graph
#    boost_program_options sot_loader ros_bridge roscpp)
#pkg_config_use_dependency(run_dynamic_graph roscpp)
#pkg_config_use_dependency(run_dynamic_graph sot_core)

###################
## add unit tests #
###################
#set(CONFIG_PATH ${PROJECT_SOURCE_DIR}/config)

##catkin_add_gtest(test_estimator_${ROBOT}
##  ../template/tests/main.cpp
##  ../template/tests/estimator_ut.cpp
##)
### link to the created librairies and its dependencies
##target_link_libraries(test_estimator_${ROBOT}
##    ${PROJECT_NAME} ${catkin_LIBRARIES})
### create 2 macro: the path to the config file and the path to the data file
##target_compile_definitions(test_estimator_${ROBOT} PUBLIC
##    TEST_PARAMETER_YAML_FILE_PATH="${CONFIG_PATH}/demos_and_tests/test_vicon_base_state_estimator.yaml"
##    TEST_DATA_FILE_PATH="${CONFIG_PATH}/demos_and_tests/test_vicon_base_state_estimator.dat"
##)

##############
## add demos #
##############

## simple code that shows how to use the estimator based on IMU and Vicon
##add_executable(
##    ${PROJECT_NAME}_demo ../template/demos/estimator_vicon_imu_demo.cpp
##)
##target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME} ${catkin_LIBRARIES})

###########################
## building documentation #
###########################
#set(BUILD_DOCUMENTATION OFF CACHE BOOL
#    "Set to ON if you want to build the documentation")
#if(BUILD_DOCUMENTATION)
#  message(STATUS "building doxygen documentation for ${PROJECT_NAME}")
#  FIND_PACKAGE(Doxygen)
#  if (NOT DOXYGEN_FOUND)
#    message(FATAL_ERROR
#        "Doxygen is needed to build the documentation. "
#        "Please install it correctly")
#  endif()

#  configure_file(Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
#  # the doxygen target is generated
#  add_custom_target (${PROJECT_NAME}_doc ALL
#  COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
#  SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)
#endif()

###############################
## manage python file install #
###############################
#catkin_python_setup()

########################################################
# manage the creation of the libraries and executables #
########################################################
add_subdirectory(src)

#########################
# manage the unit tests #
#########################
add_subdirectory(tests)

####################
# manage the demos #
####################
add_subdirectory(demos)
