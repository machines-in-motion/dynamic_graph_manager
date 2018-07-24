/**
 * \file dynamic_graph_manager.cpp
 * \brief Extended Kalman Filter
 * \author Nick Rotella
 * \date 2018
 *
 * This file declares the DynamicGraphManager class.
 * This class has for purpose to manage the different processes during run time.
 * The main tasks are:
 *   - [1] Creates the Dynamic Graph device, the python interpretor, and the
 *         Drivers
 *   - [2] Ask the python interpretor to advertize its ROS services
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
 *
 *
 * Usage: see demo_simple_integrator
 */

#ifndef DYNAMIC_GRAPH_MANAGER_HH
#define DYNAMIC_GRAPH_MANAGER_HH

# include <ros/ros.h>

namespace dynamicgraph
{

class DynamicGraphManager
{
public:

  /**
   * @brief DynamicGraphManager, constructor of the class
   */
  DynamicGraphManager();


private:
  /**
   * @brief ros_node_handle_ is used to advertize the diverse ROS services
   */
  std::unique_ptr<ros::NodeHandle> ros_node_handle_;
  /**
   * @brief ros_service_start_dg_ allows to start the dynamic graph on call.
   * It simply sets a flags that is used to wait the user call.
   */
  ros::ServiceServer ros_service_start_dg_; /*!< State dimension */
  /**
   * @brief ros_service_stop_dg_ allows to stop the dynamic graph on call.
   * It simply sets a flags that stop the main real time the control loop.
   */
  ros::ServiceServer ros_service_stop_dg_; /*!< State dimension */


};

}
#endif // DYNAMIC_GRAPH_MANAGER_HH
