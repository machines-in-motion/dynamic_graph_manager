# Define a list of the subdirectories
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Detect the parent devel folder
macro(FIND_DEVEL_FOLDER CURRENT_DIR)

  set(CURRENT_FOLDER ${CURRENT_DIR})
  set(stop_find_devel_folder OFF)

  subdirlist(SUB_FOLDERS ${CURRENT_FOLDER})
  foreach(subfold ${SUB_FOLDERS})
    if(${subfold} STREQUAL "devel")
      set(DEVEL_FOLDER ${CURRENT_FOLDER}/devel)
      set(stop_find_devel_folder ON)
    endif()
  endforeach()      

  if(NOT ${stop_find_devel_folder})
    get_filename_component(CURRENT_FOLDER ${CURRENT_FOLDER} PATH)
    find_devel_folder(${CURRENT_FOLDER})
  endif()
endmacro()

find_devel_folder(${CMAKE_CURRENT_SOURCE_DIR})
# Just declare where the plugin directory is for all dynamic graph plugins
set(DYNAMIC_GRAPH_GLOBAL_PLUGIN_DIR ${DEVEL_FOLDER}/lib/plugin)

set(DYNAMIC_GRAPH_PLUGIN_DIR ${CATKIN_DEVEL_PREFIX}/lib/plugin)

set(DYNAMIC_GRAPH_PYTHON_DIR
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/dynamic_graph_manager)
