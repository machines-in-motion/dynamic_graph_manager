######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(dynamic_graph_manager)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################

set(CATKIN_PKGS ${CATKIN_PKGS}
    # MPI-IS dependencies
    mpi_cmake_modules
    shared_memory
    real_time_tools
    yaml_cpp_catkin

    # ROS dependencies
    message_generation
    roscpp
    tf
    tf2_bullet
    realtime_tools
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

search_for_python(2.7 REQUIRED)
search_for_boost()
search_for_eigen()
search_for_pthread()

# Trigger dependency to dynamic-graph.
add_required_dependency("yaml-cpp >= 0.5")
add_required_dependency("dynamic-graph >= 3.0.0")
add_required_dependency("dynamic-graph-python >= 3.0.0")

######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

#########################################
# Declare the service files to be built #
#########################################
add_service_files(FILES
  RunCommand.srv
  RunPythonFile.srv
  TestUserCmdBool.srv
)

#########################################
# Declare the message files to be built #
#########################################
add_message_files(FILES
  Matrix.msg
  Vector.msg
)

##############################
# manage python file install #
##############################
catkin_python_setup()

#####################################################################
# Actually generate the language-specific message and service files #
#####################################################################
generate_messages(DEPENDENCIES std_msgs)

########################################################
# manage the creation of the libraries and executables #
########################################################
add_subdirectory(src)

####################
# manage the demos #
####################
add_subdirectory(demos)

#########################
# manage the unit tests #
#########################
add_subdirectory(tests)

##########################################
# export the package as a catkin package #
##########################################
catkin_package(
  LIBRARIES ${EXPORTED_LIBRAIRIES}
  INCLUDE_DIRS include ${${dg_name}_INCLUDE_DIRS} ${${dgp_name}_INCLUDE_DIRS}
  CATKIN_DEPENDS ${CATKIN_PKGS}
  CFG_EXTRAS ${CMAKE_LOCAL_MODULES}
)

###########################################################################
# Install the python files located in ```python/dynamic_graph_manager/``` #
# in ${DYNAMIC_GRAPH_PYTHON_DIR}/```""```                                 #
###########################################################################
install_python_files("" "python/dynamic_graph_manager")

##########################
# building documentation #
##########################
build_doxygen_documentation()