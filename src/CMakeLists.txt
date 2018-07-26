#############################################
## create the dynamic graph manager library #
#############################################

set(DYNAMIC_GRAPH_MANAGER_SRC
    dynamic_graph_manager.cpp
    ros_init.cpp
    ros_interpreter.cpp
)
ADD_LIBRARY(dynamic_graph_manager SHARED ${DYNAMIC_GRAPH_MANAGER_SRC})

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
