/**
 * \file dynamic_graph_manager.cpp
 * \brief Extended Kalman Filter
 * \author Nick Rotella
 * \date 2018
 *
 * Implementation of the DynamicGraphManager class
 *
 */

#include "dynamic_graph_manager.hh"

using namespace dynamicgraph;

DynamicGraphManager::DynamicGraphManager()
{
    int argc = 1;
    char* arg0 = strdup("dynamic_graph_bridge");
    char* argv[] = {arg0, 0};
    ros::init(argc, argv, "dynamic_graph_bridge");
    free (arg0);

    ros_node_handle_.reset(new ros::NodeHandle(""));

    ros_service_start_dg_ = ros_node_handle_->advertiseService(
                                "start_dynamic_graph",
                                &SotLoaderBasic::start_dg,
                                this);
    ros_service_stop_dg_ = ros_node_handle_->advertiseService(
                               "stop_dynamic_graph",
                               &SotLoaderBasic::stop_dg,
                               this);
}

bool SotLoaderBasic::start_dg(std_srvs::Empty::Request& ,
                         std_srvs::Empty::Response& )
{
  dynamic_graph_stopped_=false;
  return true;
}

bool SotLoaderBasic::stop_dg(std_srvs::Empty::Request& ,
                         std_srvs::Empty::Response& )
{
  dynamic_graph_stopped_ = true;
  return true;
}
