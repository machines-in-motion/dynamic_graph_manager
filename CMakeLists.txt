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
        *.srv *.msg
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
    message_generation
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
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
#    ${PYTHON_INCLUDE_PATH}
#    ${Eigen_INCLUDE_DIRS}
)

#########################################
# Declare the service files to be built #
#########################################
add_service_files(FILES
  RunCommand.srv
  RunPythonFile.srv
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

###########################################
## export the package as a catkin package #
###########################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${CATKIN_PKGS}
)

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

###########################
## building documentation #
###########################
set(BUILD_DOCUMENTATION OFF CACHE BOOL
    "Set to ON if you want to build the documentation")
if(BUILD_DOCUMENTATION)
  message(STATUS "building doxygen documentation for ${PROJECT_NAME}")
  FIND_PACKAGE(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
        "Doxygen is needed to build the documentation. "
        "Please install it correctly")
  endif()
  configure_file(Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
  # the doxygen target is generated
  add_custom_target (${PROJECT_NAME}_doc ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
    SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)
endif()
