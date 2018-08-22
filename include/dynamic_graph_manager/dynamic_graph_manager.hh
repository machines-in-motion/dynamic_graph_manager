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
#include <wait.h>
#include <atomic>

// used to deal with shared memory
#include <shared_memory/shared_memory.hpp>

// get the yaml configuration
#include <yaml-cpp/yaml.h>

// ROS includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_graph_manager/ros_interpreter.hh>

// some useful tools like the yaml parsing
#include <dynamic_graph_manager/tools.hh>

// the device of the dynamic-graph
#include <dynamic_graph_manager/device.hh>

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
 *              the shared std::map commands.
 * In this class we heavily depend on std::unique pointers in order to
 * initialize the DynamicGraph process and the hardware communication process
 * independently.
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
   * @brief DynamicGraphManager, destructor of the class
   */
  virtual ~DynamicGraphManager();

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
   */
  void wait_start_dynamic_graph();

  /**
   * @brief wait_stop_dynamic_graph put the current thread to sleep until the
   * user stop the dynamic graph
   */
  void wait_stop_dynamic_graph();

  /**
   * @brief wait_stop_hardware_communication put the current thread to sleep
   * until the user stop the hardware communication.
   */
  void wait_stop_hardware_communication();

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
   * @brief run_hardware_communication_process spawns the real time thread and
   * goes to sleep undefinitely
   */
  void run_hardware_communication_process();

  /***************************
   * method to be overloaded *
   ***************************/

  /**
   * @brief initialize_hardware_communication_process instanciate all variables
   * related to the hardware communication. In addition it spawns the real
   * time thread. WARNING, this function needs to be overloaded using the actual
   * drivers of the robot.
   */
  virtual void initialize_hardware_communication_process()
  {
    throw(std::runtime_error(std::string("DynamicGraphManager::") +
                             "initialize_hardware_communication_process(): " +
                             "this method needs to be overloaded"));
  }

  /**
   * @brief get_sensors_to_map is the fonction that get the motor command from
   * a map and that uses the drivers to send these command to the robot.
   * Each robot must have a different implementation of this function.
   * WARNING, this function needs to be overloaded using the actual
   * drivers of the robot.
   */
  virtual void get_sensors_to_map(VectorDGMap&)
  {
    throw(std::runtime_error(std::string("DynamicGraphManager::") +
                                         "get_sensors_to_map(VectorDGMap&): " +
                                         "this method needs to be overloaded"));
  }

  /**
   * @brief set_motor_controls_from_map is the fonction that get the motor command from
   * a map and that uses the drivers to send these command to the robot.
   * Each robot must have a different implementation of this function.
   * WARNING, this function needs to be overloaded using the actual
   * drivers of the robot.
   */
  virtual void set_motor_controls_from_map(const VectorDGMap&)
  {
    throw(std::runtime_error(std::string("DynamicGraphManager::") +
                             "set_motor_controls_from_map(const VectorDGMap&): " +
                             "this method needs to be overloaded"));
  }

  /************************
   * gettters and setters *
   ************************/

  /**
   * @brief stop_dynamic_graph stop the DynamicGraph. ;)
   */
  void stop_dynamic_graph()
  {
    is_dynamic_graph_stopped_ = true;
  }

  /**
   * @brief start_dynamic_graph start the DynamicGraph. ;)
   */
  void start_dynamic_graph()
  {
    is_dynamic_graph_stopped_ = false;
  }

  /**
   * @brief get the status of the dynamic graph (is running or not)
   * @return the flags is_dynamic_graph_stopped_ value
   */
  bool is_dynamic_graph_stopped()
  {
    return is_dynamic_graph_stopped_;
  }

  /**
   * @brief stop_hardware_communication stops the hardware communication. ;)
   */
  void stop_hardware_communication()
  {
    is_hardware_communication_stopped_ = true;
  }

  /**
   * @brief start_hardware_communication starts the hardware communication. ;)
   */
  void start_hardware_communication()
  {
    is_hardware_communication_stopped_ = false;
  }

  /**
   * @brief get the status of the hardware communication (is running or not).
   * @return the flags is_dynamic_graph_stopped_ value.
   */
  bool is_hardware_communication_stopped()
  {
    return is_hardware_communication_stopped_;
  }

  /**
   * @brief device is a getter method on the Device internal pointer.
   * @return a const reference to the device.
   */
  Device& device()
  {
    if(device_ != nullptr)
      return *device_;
    throw(std::runtime_error(std::string("DynamicGraphManager::device():Try") +
                             "accessing a device that has not been created"));
  }

  /**
   * @brief has_dynamic_graph_process_died check if the process of the
   * DynamicGraph has died or not.
   * @return true if the DynamicGraph process died.
   */
  bool has_dynamic_graph_process_died();

private:

  /**
   * @brief start_dg is the callback method of the ROS service start dynamic
   * graph.
   * @return true.
   */
  bool start_dynamic_graph(std_srvs::Empty::Request& ,
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
  bool stop_dynamic_graph(std_srvs::Empty::Request& ,
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
   * The type "atomic" is here to make sure that this variable is thread safe
   */
  std::atomic<bool> is_dynamic_graph_stopped_;

  /**
   * @brief is_hardware_communication_stopped_ is the flag reflecting the state
   * of the hardware communication thread.
   *  - TRUE the hardware communication is NOT running.
   *  - FALSE the hardware communication IS running.
   * The type "atomic" is here to make sure that this variable is thread safe
   */
  std::atomic<bool> is_hardware_communication_stopped_;

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

  /**
   * @brief pid_dynamic_graph_process_ is the pid of the DynamicGraph process.
   * It is initialized to 0 and set durning the DynamicGraphManager::run method
   */
  pid_t pid_dynamic_graph_process_;

  /**
   * @brief pid_hardware_communication_process_ is the pid of the hardware
   * communication process. It is initialized to 0 and set durning the
   * DynamicGraphManager::run method
   */
  pid_t pid_hardware_communication_process_;

  /**
   * @brief params_ is the pool of paramters in a yaml tree
   */
  YAML::Node params_;

  /**
   * @brief device_ is the DynamicGraph device that manages the computation of
   * the graph.
   */
  std::unique_ptr<Device> device_;
};

} // namespace dynamic_graph

#endif // DYNAMIC_GRAPH_MANAGER_HH
