/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Expose the ROS entities to python.
 */

#include "dynamic-graph/python/module.hh"
#include "dynamic-graph/python/signal.hh"
#include "dynamic_graph_manager/ros_entities/ros_publish.hpp"
#include "dynamic_graph_manager/ros_entities/ros_subscribe.hpp"
#include "dynamic_graph_manager/ros_entities/time_point_io.hpp"

namespace dg = dynamicgraph;

typedef bp::return_value_policy<bp::reference_existing_object>
    reference_existing_object;

BOOST_PYTHON_MODULE(ros_entities)
{
    bp::import("dynamic_graph");

    using dynamic_graph_manager::RosPublish;
    using dynamic_graph_manager::RosSubscribe;

    dynamicgraph::python::exposeEntity<RosPublish>();
    dynamicgraph::python::exposeEntity<RosSubscribe>();

    dg::python::exposeSignalsOfType<dynamic_graph_manager::timestamp_t, int>(
        "Timestamp");
}
