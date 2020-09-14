/**
 * @file main.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */
#include "simple_dgm.hpp"

int main(int, char*[])
{
    std::string yaml_params_file =
        DEMO_RESOURCES_PATH + std::string("simple_robot.yaml");
    std::cout << "Loading parameters from " << yaml_params_file << std::endl;
    YAML::Node param = YAML::LoadFile(yaml_params_file);
    dynamic_graph_manager::SimpleDGM dgm;

    dgm.initialize(param);
    dgm.run();
    while (dynamic_graph_manager::ros_ok())
    {
        dynamic_graph_manager::ros_spin(
            dynamic_graph_manager::DynamicGraphManager::hw_com_ros_node_name_);
        real_time_tools::Timer::sleep_microseconds(20);
    }
}