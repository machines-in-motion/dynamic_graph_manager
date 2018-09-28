#include "dynamic_graph_manager/ros_init.hh"
#include "ros_entities/ros_tf_listener.hh"

#include <dynamic-graph/factory.h>

namespace dynamic_graph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTfListener, "RosTfListener");
}
