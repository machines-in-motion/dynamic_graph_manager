/**
 * \file dynamic_graph_manager.cpp
 * \brief Extended Kalman Filter
 * \author Nick Rotella
 * \date 2018
 *
 * Implementation of the DynamicGraphManager class
 *
 */

#include <dynamic_graph_manager/ros_init.hh>
#include <dynamic_graph_manager/dynamic_graph_manager.hh>

using namespace dynamic_graph;

DynamicGraphManager::DynamicGraphManager():
  /* Get the node handle reference from the global variable */
  ros_node_handle_(ros_init()),
  /* Create the python_interpreter ROS wrapper */
  ros_python_interpreter_(ros_node_handle_)
{

  /* Advertize the service to start and stop the dynamic graph */
  ros_service_start_dg_ = ros_node_handle_.advertiseService(
                              "start_dynamic_graph",
                              &DynamicGraphManager::start_dg,
                              this);
  ros_service_stop_dg_ = ros_node_handle_.advertiseService(
                             "stop_dynamic_graph",
                             &DynamicGraphManager::stop_dg,
                             this);

  /* Upon construction the graph is inactive */
  is_dynamic_graph_stopped_ = true;

  /* construct the python interpreter */
  /* advertize the ros::services associated to the python interpreter */

}
