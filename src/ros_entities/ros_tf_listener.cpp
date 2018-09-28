#include "dynamic_graph_bridge/ros_init.hh"
#include "ros_tf_listener.hh"

#include <dynamic-graph/factory.h>

namespace dynamicgraph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTfListener, "RosTfListener");
}
