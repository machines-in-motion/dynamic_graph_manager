############################################
# create the dynamic graph manager library #
############################################

set(DYNAMIC_GRAPH_MANAGER_SRC
    dynamic_graph_manager.cpp
    device.cpp
    periodic-call.cpp
    ros_init.cpp
    ros_interpreter.cpp
    tools.cpp
    exception/exception-abstract.cpp
    exception/exception-factory.cpp
    exception/exception-signal.cpp
    exception/exception-tools.cpp
    exception/exception-dynamic.cpp
    exception/exception-feature.cpp
    exception/exception-task.cpp
)
add_library(dynamic_graph_manager SHARED ${DYNAMIC_GRAPH_MANAGER_SRC})
target_link_libraries(dynamic_graph_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  rt
)
set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} dynamic_graph_manager
    PARENT_SCOPE)


set(ROS_ENTITIES_SRC
  ros_entities/dg_to_ros.cpp
  ros_entities/ros_publish.cpp
  ros_entities/ros_subscribe.cpp
  ros_entities/ros_tf_listener.cpp
  ros_entities/ros_time.cpp
  ros_entities/ros_queued_subscribe.cpp
  ros_entities/converter.cpp
)
add_library(ros_entities SHARED ${ROS_ENTITIES_SRC})
target_link_libraries(ros_entities
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dynamic_graph_manager
  rt
)
set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} ros_entities PARENT_SCOPE)


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
