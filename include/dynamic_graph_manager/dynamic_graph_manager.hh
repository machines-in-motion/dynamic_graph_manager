/**
 * \file dynamic_graph_manager.hh
 * \brief Extended Kalman Filter
 * \author Nick Rotella
 * \date 2018
 *
 * This file declares the DynamicGraphManager class.
 * usage: see demos and unit tests and documentation
 */

#ifndef DYNAMIC_GRAPH_MANAGER_HH
#define DYNAMIC_GRAPH_MANAGER_HH

// ROS includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_graph_manager/ros_interpreter.hh>

namespace dynamic_graph
{
/**
 * This class has for purpose to manage the different processes during run time.
 * The main tasks are:
 *   - [1] Creates the Dynamic Graph device, the python interpreter, and the
 *         Drivers
 *   - [2] Ask the python interpreter to advertize its ROS services
 *   - [3] Ask the drivers to initialize the communucation with the hardware
 *   - [4] Loads a yaml/urdf config file.
 *   - [5] Advertize the ROS services start/stop dynamic graph
 *   - [6] Wait for the ROS service start dynamic graph to be called
 *   - [7] Spawn the first real time process that executes the following:
 *      - [7.1] gets  the  sensor  data  using Drivers and  saves  them  in  the
 *              shared std::map sensors
 *      - [7.2] reads the control values in the shared std::map commands and
 *              send them to the motors via the Drivers
 *   - [8] Spawn the second real time process that executes the following:
 *      - [8.1] passes the std::map sensors to the Device, which copies the data
 *              to its output signals
 *      - [8.2] gets the control values from the Device (which triggers the
 *              evaluation of  the  dynamic  graph)  and  copies  them  into
 *              the  shared std::map commands
 */
class DynamicGraphManager
{
  /** Public methods */
public:

  /**
   * @brief DynamicGraphManager, constructor of the class
   */
  DynamicGraphManager();

  /**
   * @brief get the status of the dynamic graph (is running or not)
   * @return the flags is_dynamic_graph_stopped_ value
   */
  bool is_dynamic_graph_stopped()
  {
    return is_dynamic_graph_stopped_;
  }

  /** Private methods */
private:

  /**
   * @brief start_dg is the callback method of the ROS service start dynamic
   * graph
   * @return True
   */
  bool start_dg(std_srvs::Empty::Request& ,
                std_srvs::Empty::Response& )
  {
    is_dynamic_graph_stopped_ = false;
    return true;
  }

  /**
   * @brief stop_dg is the callback method of the ROS service stop dynamic
   * graph
   * @return
   */
  bool stop_dg(std_srvs::Empty::Request& ,
               std_srvs::Empty::Response& )
  {
    is_dynamic_graph_stopped_ = true;
    return true;
  }

  /** Private attributes */
private:
  /**
   * @brief ros_node_handle_ is reference to the ros::NodeHandle used to advertize
   * the ros::services
   */
  ros::NodeHandle& ros_node_handle_;
  /**
   * @brief ros_service_start_dg_ allows to start the dynamic graph on call.
   * It simply sets a flags that is used to wait the user call.
   */
  ros::ServiceServer ros_service_start_dg_;
  /**
   * @brief ros_service_stop_dg_ allows to stop the dynamic graph on call.
   * It simply sets a flags that stop the main real time the control loop.
   */
  ros::ServiceServer ros_service_stop_dg_;
  /**
   * @brief is_dynamic_graph_stopped_ is the flag reflecting the state of the
   * dynamic graph.
   *  - TRUE the Dynamic Graph is NOT running.
   *  - FALSE the Dynamic Graph IS running.
   */
  bool is_dynamic_graph_stopped_;
  /**
   * @brief interpreter_ is a ROS wrapper around a python interpreter
   */
  dynamic_graph::RosPythonInterpreter ros_python_interpreter_;

};

} // namespace dynamic_graph

#endif // DYNAMIC_GRAPH_MANAGER_HH
