/**
 * @file
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 */

#include "dynamic_graph_manager/ros_python_interpreter_server.hpp"

int main()
{
    dynamic_graph_manager::RosPythonInterpreterServer python_server;
    python_server.start_ros_service();

    dynamic_graph_manager::ros_spin();
    dynamic_graph_manager::ros_shutdown();
}