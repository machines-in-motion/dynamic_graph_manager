# NORMALIZE_PATH
# ------------------
#
# Convert the windows style path into unix style path.
#
# On windows, the folder separator is \, wihch can lead to some issues
# because of the appearance of special characters like \p, \n ...
#
FUNCTION(NORMALIZE_PATH mypath)
  IF(WIN32)
    STRING(REPLACE "\\" "/" ${mypath} "${${mypath}}")
    SET(${mypath} ${${mypath}} PARENT_SCOPE)
  ENDIF(WIN32)
ENDFUNCTION(NORMALIZE_PATH)


# FIND_PYTHON
# -----------
#
# Define a macro that correctly look for python between 2.7 and 3
#
MACRO(SEARCH_FOR_PYTHON)

FIND_PACKAGE(PythonInterp ${ARGN})
IF (NOT ${PYTHONINTERP_FOUND} STREQUAL TRUE)
   MESSAGE(FATAL_ERROR "Python executable has not been found.")
ENDIF (NOT ${PYTHONINTERP_FOUND} STREQUAL TRUE)
MESSAGE(STATUS "PythonInterp: ${PYTHON_EXECUTABLE}")

FIND_PACKAGE(PythonLibs ${ARGN})
MESSAGE(STATUS "PythonLibraries: ${PYTHON_LIBRARIES}")
IF (NOT ${PYTHONLIBS_FOUND} STREQUAL TRUE)
   MESSAGE(FATAL_ERROR "Python has not been found.")
ENDIF (NOT ${PYTHONLIBS_FOUND} STREQUAL TRUE)

# Find PYTHON_LIBRARY_DIRS
GET_FILENAME_COMPONENT(PYTHON_LIBRARY_DIRS "${PYTHON_LIBRARIES}" PATH)
MESSAGE(STATUS "PythonLibraryDirs: ${PYTHON_LIBRARY_DIRS}")
MESSAGE(STATUS "PythonLibVersionString: ${PYTHONLIBS_VERSION_STRING}")

# Default Python packages directory
SET(PYTHON_PACKAGES_DIR site-packages)

# Use either site-packages (default) or dist-packages (Debian packages) directory
OPTION(PYTHON_DEB_LAYOUT "Enable Debian-style Python package layout" OFF)

IF (PYTHON_DEB_LAYOUT)
  SET(PYTHON_PACKAGES_DIR dist-packages)
ENDIF (PYTHON_DEB_LAYOUT)

EXECUTE_PROCESS(
  COMMAND "${PYTHON_EXECUTABLE}" "-c"
  "import sys, os; print(os.sep.join(['lib', 'python' + sys.version[:3], '${PYTHON_PACKAGES_DIR}']))"
  OUTPUT_VARIABLE PYTHON_SITELIB
  ERROR_QUIET)

# Remove final \n of the variable PYTHON_SITELIB
STRING(REPLACE "\n" "" PYTHON_SITELIB "${PYTHON_SITELIB}")
NORMALIZE_PATH(PYTHON_SITELIB)

# Log Python variables
LIST(APPEND LOGGING_WATCHED_VARIABLES
  PYTHONINTERP_FOUND
  PYTHONLIBS_FOUND
  PYTHON_LIBRARY_DIRS
  PYTHONLIBS_VERSION_STRING
  PYTHON_EXECUTABLE
  )

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

ENDMACRO(SEARCH_FOR_PYTHON)


#.rst:
# .. command:: DYNAMIC_GRAPH_PYTHON_MODULE ( SUBMODULENAME LIBRARYNAME TARGETNAME )
#
#   Add a python submodule to dynamic_graph
#
#   :param SUBMODULENAME: the name of the submodule (can be foo/bar),
#
#   :param LIBRARYNAME:   library to link the submodule with.
#
#  .. note::
#    Before calling this macro, set variable NEW_ENTITY_CLASS as
#    the list of new Entity types that you want to be bound.
#    Entity class name should match the name referencing the type
#    in the factory.
#
MACRO(DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGET_NAME)

  set(SUBMODULE_DIR ${SUBMODULENAME})
  string(REPLACE "-" "_" SUBMODULE_DIR "${SUBMODULE_DIR}")

  IF(NOT DEFINED PYTHONLIBS_FOUND)
    SEARCH_FOR_PYTHON()
  ELSEIF(NOT ${PYTHONLIBS_FOUND} STREQUAL "TRUE")
    MESSAGE(FATAL_ERROR "Python has not been found.")
  ENDIF()

  # local var to create the destination folders and install it
  SET(OUTPUT_MODULE_DIR ${DYNAMIC_GRAPH_PYTHON_DIR}/${SUBMODULE_DIR})

  # create the install folder
  FILE(MAKE_DIRECTORY ${OUTPUT_MODULE_DIR})


  # We need to set this policy to old to accept wrap target.
  CMAKE_POLICY(PUSH)
  IF(POLICY CMP0037)
    CMAKE_POLICY(SET CMP0037 OLD)
  ENDIF()

  # create the library
  SET(PYTHON_MODULE "${TARGET_NAME}")
  ADD_LIBRARY(${PYTHON_MODULE}
    MODULE
    ${PROJECT_SOURCE_DIR}/cmake/python-module-py.cc
  )
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${LIBRARYNAME} ${PYTHON_LIBRARY})
  SET_TARGET_PROPERTIES(${PYTHON_MODULE} PROPERTIES
    PREFIX ""
    OUTPUT_NAME wrap
    LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_MODULE_DIR}/"
  )

  CMAKE_POLICY(POP)

  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/__init__.py.in
    ${OUTPUT_MODULE_DIR}/__init__.py
  )

  SET(ENTITY_CLASS_LIST "")
  FOREACH (ENTITY ${NEW_ENTITY_CLASS})
    SET(ENTITY_CLASS_LIST "${ENTITY_CLASS_LIST}${ENTITY}('')\n")
  ENDFOREACH(ENTITY ${NEW_ENTITY_CLASS})

ENDMACRO(DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGET_NAME)


#.rst:
# .. command:: INSTALL_PYTHON_FILES (SUBMODULENAME PYTHON_SRC_DIR)
#
#   Install python files in the python submodule of dynamic_graph
#
#   :param PYTHON_SRC_DIR is the root of the python module. It *must* contain
#   a __init__.py. All file bleow this folder will be install in the python
#   submodule
#
#   :param SUBMODULENAME: the name of the submodule (can be foo/bar),
#
#   :param LIBRARYNAME:   library to link the submodule with.
#
#  .. note::
#    Before calling this macro, set variable NEW_ENTITY_CLASS as
#    the list of new Entity types that you want to be bound.
#    Entity class name should match the name referencing the type
#    in the factory.
#
MACRO(INSTALL_PYTHON_FILES SUBMODULENAME PYTHON_SRC_DIR)
  # get the global path
  get_filename_component(PYTHON_SRC_FULL_PATH ${PYTHON_SRC_DIR} ABSOLUTE)

  # local var to create the destination folders and install it
  set(OUTPUT_MODULE_DIR ${DYNAMIC_GRAPH_PYTHON_DIR}/${SUBMODULENAME})

  # create the install folder
  file(MAKE_DIRECTORY ${OUTPUT_MODULE_DIR})

  # fetch all python files with a relative path to the PYTHON_SRC_FULL_PATH
  file(GLOB_RECURSE PYTHON_FILES RELATIVE "${PYTHON_SRC_FULL_PATH}"
    "${PYTHON_SRC_FULL_PATH}/*.py")

  foreach (PYTHON_FILE ${PYTHON_FILES})
    STRING(REGEX REPLACE "-" "_" PYTHON_FILE_STANDARDIZED ${PYTHON_FILES})
    configure_file(${PYTHON_SRC_FULL_PATH}/${PYTHON_FILE}
      ${OUTPUT_MODULE_DIR}/${PYTHON_FILE} @ONLY)
  endforeach ()

  # message(WARNING "${PYTHON_FILES}")

ENDMACRO(INSTALL_PYTHON_FILES PYTHON_SRC_DIR)
