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

ENDMACRO(FIND_PYTHON)
