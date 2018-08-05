/**
 * \file dynamic_graph_manager.hh
 * \brief The Control Manager
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file declares the DynamicGraphManager class.
 * This code manages the different threads and processes in order to use the
 * DynamicGrpah.
 * usage: see demos and unit tests and documentation
 */

#ifndef DYNAMIC_GRAPH_MANAGER_HH
#define DYNAMIC_GRAPH_MANAGER_HH

// used to spawn the real time thread
#include <thread>

// used to deal with shared memory
#include <boost/interprocess/managed_shared_memory.hpp>
namespace bipc = boost::interprocess;

// get the yaml configuration
#include <yaml-cpp/yaml.h>

// ROS includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_graph_manager/ros_interpreter.hh>

namespace dynamic_graph
{

/**
 * \brief SHARED_MEMORY_NAME is the name used by boost to retrieve the shared
 * memory
 */
#define SHARED_MEMORY_NAME "DGM_Shared_memory"

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
  /*******************
   *  Public methods *
   *******************/
public:

  /**
   * @brief DynamicGraphManager, constructor of the class
   */
  DynamicGraphManager();

  /**
   * @brief initialize the basic variables
   */
  void initialize(YAML::Node param);

  /**
   * @brief run() splits the process in the dynamic_graph process and the
   * hadware_communication process. It initialize them and run them. WARNING
   * this a blocking function
   */
  void run();

  /**
   * @brief wait_start_dynamic_graph put the current thread to sleep until the
   * user start the dynamic graph
   * @return True: dynamic graph started before the watch dog expires,
   *         False: otherwize
   */
  void wait_start_dynamic_graph();

  /**
   * @brief initialize_dynamic_graph_process instanciates all variables related
   * to the dynamic_graph and user interface.
   */
  void initialize_dynamic_graph_process();

  /**
   * @brief run_dynamic_graph_process spawns the real time thread and becomes
   * a ros spinner (thread in charge of the ros::service callbacks)
   */
  void run_dynamic_graph_process();

  /**
   * @brief initialize_hardware_communication_process instanciate all variables
   * related to the hardware communication. In addition it spawns the real
   * time thread.
   */
  void initialize_hardware_communication_process();

  /**
   * @brief run_hardware_communication_process spawns the real time thread and
   * goes to sleep undefinitely
   */
  void run_hardware_communication_process();

  /************************
   * gettters and setters *
   ************************/

  /**
   * @brief get the status of the dynamic graph (is running or not)
   * @return the flags is_dynamic_graph_stopped_ value
   */
  bool is_dynamic_graph_stopped()
  {
    return is_dynamic_graph_stopped_;
  }

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

  /**
   * @brief start_ros_service is the method that advertize the different ros
   * services
   */
  void start_ros_service(ros::NodeHandle& ros_node_handle);

  /**
   * @brief dynamic_graph_real_time_loop is the method used to execute the
   * dynamic graph
   */
  void dynamic_graph_real_time_loop();

  /**
   * @brief hardware_communication_real_time_loop is the method that communicate
   * with the hardware and send the commands (torque, position, current, ...)
   */
  void hardware_communication_real_time_loop();

  /***********************
   *  Private attributes *
   ***********************/
private:
  /**
   * @brief ros_service_start_dg_ allows to start the dynamic graph on call.
   * It simply sets a flags that is used to wait the user call. Only used in
   * the dynamic_graph process.
   */
  ros::ServiceServer ros_service_start_dg_;

  /**
   * @brief ros_service_stop_dg_ allows to stop the dynamic graph on call.
   * It simply sets a flags that stop the main real time the control loop.
   * Only used in the dynamic_graph process.
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
   * @brief ros_python_interpreter_ptr_ is a ROS wrapper around a python
   * interpreter.
   */
  std::unique_ptr<dynamic_graph::RosPythonInterpreter> ros_python_interpreter_;

  /**
   * @brief thread_dynamic_graph_ is the real time thread that runs the dynamic
   * graph.
   */
  std::unique_ptr<std::thread> thread_dynamic_graph_;

  /**
   * @brief thread_hardware_communication_ is the real thread that communicate
   * with the hardware.
   */
  std::unique_ptr<std::thread> thread_hardware_communication_;


  /***********************
   *  Pool of parameters *
   ***********************/

  YAML::Node params_;

  /***********************************
   * management of the shared memory *
   * *********************************/

  //Remove shared memory on construction and destruction
  struct shm_remove
  {
     shm_remove() {  bipc::shared_memory_object::remove(SHARED_MEMORY_NAME); }
     ~shm_remove(){  bipc::shared_memory_object::remove(SHARED_MEMORY_NAME); }
  } shared_memory_remover;
};

} // namespace dynamic_graph

#endif // DYNAMIC_GRAPH_MANAGER_HH
