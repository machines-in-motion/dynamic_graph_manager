#############################################
## create the dynamic graph manager library #
#############################################

set(DYNAMIC_GRAPH_MANAGER_SRC
    dynamic_graph_manager.cpp
    device.cpp
    periodic-call.cpp
    ros_init.cpp
    ros_interpreter.cpp
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
)
set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} dynamic_graph_manager
    PARENT_SCOPE)

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
