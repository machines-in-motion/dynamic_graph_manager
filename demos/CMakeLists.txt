#####################
# create some demos #
#####################

include_directories(${PROJECT_SOURCE_DIR}/demos)

#### demo with a real single motor ####
add_executable(demo_single_motor
  demo_single_motor.cpp
  single_motor/dgm_single_motor.cpp
)
target_link_libraries(demo_single_motor
  dynamic_graph_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  rt
)
target_compile_definitions(demo_single_motor PUBLIC
  YAML_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/single_motor/dgm_single_motor_simu.yaml")

#### demo with a simulated single motor ####
add_executable(demo_single_motor_simu
  demo_single_motor.cpp
  single_motor/dgm_single_motor.cpp
)
target_link_libraries(demo_single_motor_simu
  dynamic_graph_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  rt
)
target_compile_definitions(demo_single_motor_simu PUBLIC
  YAML_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/single_motor/dgm_single_motor_simu.yaml")
