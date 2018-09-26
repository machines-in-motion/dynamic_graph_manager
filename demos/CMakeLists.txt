#####################
# create some demos #
#####################

include_directories(${PROJECT_SOURCE_DIR}/demos)

# demo with a single motor simu and real robot
add_executable(demo_single_motor
  demo_single_motor.cpp
  single_motor/dgm_single_motor.cpp
)
target_link_libraries(demo_single_motor dynamic_graph_manager)
target_compile_definitions(demo_single_motor PUBLIC
  YAML_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/config/demo_single_motor.yaml)
