/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Demo of the DynamicGraphController API.
 *
 * @example demo_dynamic_graph_controller.cpp
 * In this demo we show the usage of the DynamicGraphController API.
 * In order to run it execute:
 * `rosrun dynamic_graph_manager demo_dynamic_graph_controller`.
 */
#include "dynamic_graph_manager/dynamic_graph_controller.hpp"

int main(int, char* [])
{
    // Cleanup the shared memory
    dynamic_graph_manager::dgm_shared_memory::clean();

    std::string yaml_params_file =
        TEST_CONFIG_PATH + std::string("simple_robot.yaml");
    std::cout << "Loading paramters from " << yaml_params_file << std::endl;
    YAML::Node param = YAML::LoadFile(yaml_params_file);
    dynamic_graph_manager::DynamicGraphController dg_ctrl;

    dg_ctrl.initialize(param);
    dg_ctrl.run();
    dg_ctrl.wait_until_controller_stops();
    std::cout << "End of main!" << std::endl;
}