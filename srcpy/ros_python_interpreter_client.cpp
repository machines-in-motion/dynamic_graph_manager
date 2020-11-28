/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the StepperHead class
 */

#include "dynamic_graph_manager/ros_python_interpreter_client.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

using namespace dynamic_graph_manager;

void bind_ros_python_interpreter_client(pybind11::module &module)
{
    pybind11::class_<RosPythonInterpreterClient>(module,
                                                 "RosPythonInterpreterClient")
        .def(pybind11::init<>())

        // Public methods.
        .def("run_python_command",
             &RosPythonInterpreterClient::run_python_command,
             pybind11::return_value_policy::copy)
        .def("run_python_script",
             &RosPythonInterpreterClient::run_python_script,
             pybind11::return_value_policy::copy);
}
