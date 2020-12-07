/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the dynamic_graph_manager namespace.
 */

#include <pybind11/pybind11.h>

/**
 * @brief Bindings for the RosPythonInterpreterClient class.
 *
 * @param module
 */
void bind_ros_python_interpreter_client(pybind11::module &module);

/**
 * Python bindings for the dynamic_graph_manager namespace.
 */
PYBIND11_MODULE(dynamic_graph_manager_cpp_bindings, m)
{
    // Docs.
    m.doc() = R"pbdoc(
        dynamic_graph_manager python bindings
        -------------------------------------
        .. currentmodule:: dynamic_graph_manager_cpp_bindings
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";
    // List of all the bindings.
    bind_ros_python_interpreter_client(m);
    bind_dgm_head(m);
}
