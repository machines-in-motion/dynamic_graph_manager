/**
 * @file
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 */

#include "dynamic_graph_manager/dynamic_graph_manager.hpp"

int main()
{
    std::string yaml_params_file =
        DEMO_RESOURCES_PATH + std::string("simple_robot.yaml");
    std::cout << "Loading parameters from " << yaml_params_file << std::endl;
    YAML::Node param = YAML::LoadFile(yaml_params_file);

    dynamic_graph_manager::DynamicGraphManager dgm;
    dgm.initialize(param);
    dgm.initialize_dynamic_graph_process();
    dgm.run_dynamic_graph_process();
    dynamic_graph_manager::ros_spin();
    dynamic_graph_manager::ros_shutdown();
}
