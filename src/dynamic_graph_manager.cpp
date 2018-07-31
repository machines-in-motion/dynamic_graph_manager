/**
 * \file dynamic_graph_manager.cpp
 * \brief Extended Kalman Filter
 * \author Nick Rotella
 * \date 2018
 *
 * Implementation of the DynamicGraphManager class
 *
 */

#include <unistd.h>
#include <signal.h>
#include <dynamic_graph_manager/ros_init.hh>
#include <dynamic_graph_manager/dynamic_graph_manager.hh>

using namespace dynamic_graph;

DynamicGraphManager::DynamicGraphManager():
  // Get the node handle reference from the global variable
  ros_node_handle_(ros_init()),
  ros_python_interpreter_(ros_node_handle_)
{
  // Advertize the service to start and stop the dynamic graph
  ros_service_start_dg_ = ros_node_handle_.advertiseService(
                              "start_dynamic_graph",
                              &DynamicGraphManager::start_dg,
                              this);
  ros_service_stop_dg_ = ros_node_handle_.advertiseService(
                             "stop_dynamic_graph",
                             &DynamicGraphManager::stop_dg,
                             this);
  // advertize the ros::services associated to the python interpreter
  ros_python_interpreter_.start_ros_service();
}

void DynamicGraphManager::initialize(/* yaml node */){

  // Upon construction the graph is inactive
  is_dynamic_graph_stopped_ = true;

  read_param();

  // TODO find a way to reset the dynamic graph properly (Through the device?)

}

void DynamicGraphManager::read_param()
{
  // TODO: read a yaml file
  // potentially create another fonction with the ros param?
  dt_dg_ = 0.001;
  dt_ctrl_ = 0.001;
}

void DynamicGraphManager::wait_start_dynamic_graph()
{
  while(is_dynamic_graph_stopped_)
  {
    usleep(1000);
  }
}

void DynamicGraphManager::start_real_time_loops()
{
  thread_hardware_communication_.reset(new std::thread(
        &DynamicGraphManager::dynamic_graph_real_time_loop, this));
  thread_hardware_communication_->detach();

  thread_dynamic_graph_.reset(new std::thread(
        &DynamicGraphManager::hardware_communication_real_time_loop, this));
  thread_dynamic_graph_->detach();
}

void DynamicGraphManager::dynamic_graph_real_time_loop()
{
  try{
    int * a = nullptr;
    *a = 0.0;
  }
  catch(...)
  {

  }
}

void DynamicGraphManager::hardware_communication_real_time_loop()
{

}
