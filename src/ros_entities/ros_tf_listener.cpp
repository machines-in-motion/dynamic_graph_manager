/**
 * @file ros_tf_listener.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include "ros_entities/ros_tf_listener.hpp"
#include "dynamic_graph_manager/ros_init.hpp"

#include <dynamic-graph/factory.h>

namespace dynamic_graph_manager
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTfListener, "RosTfListener");
}
