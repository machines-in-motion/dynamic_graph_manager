/**
 * @file dgm_head.hpp
 * @author Julian Viereck <jviereck@tuebingen.mpg.de>
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-07
 */

#include "dynamic_graph_manager/dgm_head.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

using namespace dynamic_graph_manager;

void bind_dgm_head(pybind11::module &module)
{
    pybind11::class_<DGMHead>(module, "DGMHead")
        .def(pybind11::init<std::string &>())

        .def("get_sensor", &DGMHead::get_sensor,
            pybind11::return_value_policy::reference_internal)
        .def("set_control", &DGMHead::set_control)

        .def("read", &DGMHead::read)
        .def("put_notify_wait", &DGMHead::put_notify_wait);
}

