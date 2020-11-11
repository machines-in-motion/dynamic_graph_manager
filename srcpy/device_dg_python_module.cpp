/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Expose the Device and the periodic call to python.
 */

#include "dynamic-graph/python/module.hh"
#include "dynamic-graph/python/signal.hh"
#include "dynamic_graph_manager/device.hpp"
#include "dynamic_graph_manager/device_simulator.hpp"
#include "dynamic_graph_manager/ros_entities/time_point_io.hpp"

namespace dg = dynamicgraph;

typedef bp::return_value_policy<bp::reference_existing_object>
    reference_existing_object;

BOOST_PYTHON_MODULE(device)
{
    bp::import("dynamic_graph");

    using dynamic_graph_manager::PeriodicCall;
    bp::class_<PeriodicCall, boost::noncopyable>("PeriodicCall", bp::no_init)
        .def("addSignal",
             static_cast<void (PeriodicCall::*)(const std::string &,
                                                dg::SignalBase<int> &)>(
                 &PeriodicCall::addSignal),
             "Add the signal to the refresh list",
             (bp::arg("name"), "signal"))
        .def("addSignal",
             static_cast<void (PeriodicCall::*)(const std::string &)>(
                 &PeriodicCall::addSignal),
             "Add the signal to the refresh list",
             (bp::arg("signal_name")))

        .def("addDownsampledSignal",
             static_cast<void (PeriodicCall::*)(const std::string &,
                                                dg::SignalBase<int> &,
                                                const unsigned int &)>(
                 &PeriodicCall::addDownsampledSignal),
             "Add the signal to the refresh list\n"
             "The downsampling factor: 1 means every time, "
             "2 means every other time, etc...",
             (bp::arg("name"), "signal", "factor"))
        .def("addDownsampledSignal",
             static_cast<void (PeriodicCall::*)(const std::string &,
                                                const unsigned int &)>(
                 &PeriodicCall::addDownsampledSignal),
             "Add the signal to the refresh list\n"
             "The downsampling factor: 1 means every time, "
             "2 means every other time, etc...",
             (bp::arg("signal_name"), "factor"))

        .def("rmSignal",
             &PeriodicCall::rmSignal,
             "Remove the signal to the refresh list",
             bp::arg("name"))
        .def("clear",
             &PeriodicCall::clear,
             "Clear all signals and commands from the refresh list.")
        .def(
            "__str__", +[](const PeriodicCall &e) {
                std::ostringstream os;
                e.display(os);
                return os.str();
            });

    using dynamic_graph_manager::Device;
    dynamicgraph::python::exposeEntity<Device>()
        .add_property("after",
                      bp::make_function(&Device::get_periodic_call_after,
                                        reference_existing_object()))
        .def_readonly("before",
                      bp::make_function(&Device::get_periodic_call_before,
                                        reference_existing_object()));
}
