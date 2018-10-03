############################################
# create the dynamic graph manager library #
############################################

set(EXCEPTION_SRC
  exception/exception-abstract.cpp
  exception/exception-factory.cpp
  exception/exception-signal.cpp
  exception/exception-tools.cpp
  exception/exception-dynamic.cpp
  exception/exception-feature.cpp
  exception/exception-task.cpp
)

set(DYNAMIC_GRAPH_MANAGER_SRC
    dynamic_graph_manager.cpp
    device.cpp
    periodic-call.cpp
    ros_init.cpp
    ros_interpreter.cpp
    tools.cpp
    ${EXCEPTION_SRC}
)
add_library(dynamic_graph_manager SHARED ${DYNAMIC_GRAPH_MANAGER_SRC})
target_link_libraries(dynamic_graph_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  rt
)
set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} dynamic_graph_manager
    PARENT_SCOPE)

################################################################
# Export some default entities used for ROS<->DG communication #
################################################################
# first we create small library for type convertions
add_library(dg_ros_convertion
  ros_entities/dg_to_ros.cpp
  ros_entities/converter.cpp
)
target_link_libraries(dg_ros_convertion dynamic_graph_manager)
set_target_properties(dg_ros_convertion PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${DYNAMIC_GRAPH_PLUGIN_DIR}"
)

# then we build the different entities and install them in devel/lib/plugin folder
set(LIST_ROS_ENTITIES
  ros_publish
  ros_subscribe
  ros_queued_subscribe
  ros_tf_listener
  ros_time
)

foreach(ROS_ENTITY ${LIST_ROS_ENTITIES})
  add_library(${ROS_ENTITY} SHARED
    ros_entities/${ROS_ENTITY}.cpp
  )
  target_link_libraries(${ROS_ENTITY} dg_ros_convertion dynamic_graph_manager)
  set_target_properties(${ROS_ENTITY} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${DYNAMIC_GRAPH_PLUGIN_DIR}
  )
  dynamic_graph_python_module("ros/${ROS_ENTITY}"
    ${ROS_ENTITY}
    ${ROS_ENTITY}_wrap
  )
endforeach()
