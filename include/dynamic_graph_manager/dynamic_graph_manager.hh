/**
 * \file dynamic_graph_manager.hh
 * \brief The Control Manager
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file declares the DynamicGraphManager class.
 * This code manages the different threads and processes in order to use the
 * DynamicGraph.
 * usage: see demos and unit tests and documentation
 */

#ifndef DYNAMIC_GRAPH_MANAGER_HH
#define DYNAMIC_GRAPH_MANAGER_HH

// used to join the different processes
#include <unistd.h>
#ifndef __APPLE__
#include <wait.h>
#endif // __APPLE__

// use of std::bind
 #include <functional>

// used to synchronise the control loop
#include <chrono>

// here this is used to use atomic (thread safe objects)
#include <atomic>

// get the yaml configuration
#include <yaml-cpp/yaml.h>

// ROS includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// used to spawn the real time thread
#include "real_time_tools/realtime_thread_creation.hpp"

// use the realtime spinner to time the loops
#include <real_time_tools/spinner.hpp>

// time measurement
#include "real_time_tools/timer.hpp"

// used to deal with shared memory
#include "shared_memory/thread_synchronisation.hpp"

// import the python interpreter ros binding
#include "dynamic_graph_manager/ros_interpreter.hh"

// some useful tools like the yaml parsing
#include "dynamic_graph_manager/tools.hh"

// the device of the dynamic-graph
#include "dynamic_graph_manager/device.hh"

namespace dynamic_graph
{

  /**
   * @brief clock is the std::chrono::high_resolution_clock object. This typedef
   * is here as a shortcut to simplify the code readability.
   */
  typedef std::chrono::steady_clock clock;

/**
 * This class has for purpose to manage the different processes during run time.
 * The main tasks are:
 *   - [1] Creates the Dynamic Graph device, the python interpreter, and the
 *         Drivers
 *   - [2] Ask the python interpreter to advertise its ROS services
 *   - [3] Ask the drivers to initialize the communication with the hardware
 *   - [4] Loads a yaml/urdf config file.
 *   - [5] Advertise the ROS services start/stop dynamic graph
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
   * this a NONE blocking function. One can spin endlessly using the ROS:
   * ros::waitForShutdown(), for example.
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
   * @brief run_python_command
   * @param file is the logging file to log the entry
   * @param command is the python command itself
   */
  void run_python_command(std::ostream& file,
                          const std::string& command);

  /**
   * @brief python_prologue get the pointer of the device in the the python
   * interpretor.
   */
  void python_prologue();

  /**
   * @brief run_dynamic_graph_process spawns the real time thread and becomes
   * a ros spinner (thread in charge of the ros::service callbacks).
   * This function is virtual has it might differ from os to os.
   */
  virtual void run_dynamic_graph_process();

  /**
   * @brief run_hardware_communication_process spawns the real time thread.
   * WARNING this function is not blocking. Function to block are available
   * like ros::waitForShutdown() for example.
   * This function is virtual has it might differ from os to os.
   */
  virtual void run_hardware_communication_process();

  /**
   * @brief run_single_process spawns the real time thread.
   * WARNING this function is not blocking. Function to block are available
   * like ros::waitForShutdown() for example.
   * This function is virtual has it might differ from os to os.
   */
  virtual void run_single_process();

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

  /**
   * @brief compute_safety_controls computes safety controls very fast in case
   * the dynamic graph is taking to much computation time or has crashed.
   */
  virtual void compute_safety_controls();

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
   * @return the flag is_dynamic_graph_stopped_ value
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
   * @brief pid_dynamic_graph_process is an accessor on the pid of the process
   * @return the pid of the dynamic graph process
   */
  pid_t pid_dynamic_graph_process()
  {
    return pid_dynamic_graph_process_;
  }

  /**
   * @brief pid_hardware_communication_process is an accessor on the pid of
   * the process
   * @return the pid of the dynamic graph process
   */
  pid_t pid_hardware_communication_process()
  {
    return pid_hardware_communication_process_;
  }

  /**
   * @brief device is a getter method on the Device internal pointer.
   * @return a const reference to the device.
   */
  Device& device()
  {
    if(device_ != nullptr)
      return *device_;
    throw(std::runtime_error("DynamicGraphManager::device():Try"
                             "accessing a device that has not been created"));
  }

  /**
   * @brief has_dynamic_graph_process_died check if the process of the
   * DynamicGraph has died or not.
   * @return true if the DynamicGraph process died.
   */
  bool has_dynamic_graph_process_died();

  /**
   * @brief is_in_safety_mode check if the dynamic graph is still alive and
   * sending commands at a descent frequency.
   * @return true if there is a problem
   */
  bool is_in_safety_mode()
  {
    return (missed_control_count_ >= max_missed_control_) ||
        has_dynamic_graph_process_died();
  }

  /**
   * @brief dg_ros_node_name_ this is the ros node name of the dynamic graph
   * process
   */
  static const std::string dg_ros_node_name_;

  /**
   * @brief hw_com_ros_node_name_ this is the ros node name of the harware
   * communication process
   */
  static const std::string hw_com_ros_node_name_;

  /**
   * @brief shared_memory_name is the name of the shared memory segment to be
   * used
   */
  static const std::string shared_memory_name_;

  /**
   * @brief sensors_map_name is the name of the sensor map inside the shared
   * memory segment
   */
  static const std::string sensors_map_name_;

  /**
   * @brief motor_controls_map_name is the name of the motor controls map inside
   * the shared memory segment
   */
  static const std::string motor_controls_map_name_;

  /**
   * @brief cond_var_sensors_name_ is the name of the condition variable in the
   * shared memory
   */
  static const std::string cond_var_name_;

private:

  /**
   * @brief start_dg is the callback method of the ROS service start dynamic
   * graph.
   * @return true.
   */
  bool start_dynamic_graph(std_srvs::Empty::Request& ,
                           std_srvs::Empty::Response& )
  {
    start_dynamic_graph();
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
    stop_dynamic_graph();
    return true;
  }

  /**
   * @brief start_ros_service is the method that advertise the different ros
   * services.
   */
  void start_ros_service(ros::NodeHandle& ros_node_handle);

  /**
   * @brief dynamic_graph_real_time_loop is the method used to execute the
   * dynamic graph.
   */
  void* dynamic_graph_real_time_loop();

  /**
   * @brief dynamic_graph_real_time_loop_helper is a static member allowing to
   * use the posix pthread_create.
   * @param context is the DynamicGraphManager that spawned the thread.
   * @return nothing interesting for us.
   */
  static void* dynamic_graph_real_time_loop_helper(void *context)
  {
    return static_cast<DynamicGraphManager *>(context)->
        dynamic_graph_real_time_loop();
  }

  /**
   * @brief hardware_communication_real_time_loop is the method that communicate
   * with the hardware and send the commands (torque, position, current, ...)
   */
  void* hardware_communication_real_time_loop();

  /**
   * @brief dynamic_graph_real_time_loop_helper is a static member allowing to
   * use the posix pthread_create.
   * @param context is the DynamicGraphManager that spawned the thread.
   * @return nothing interesting for us.
   */
  static void* hardware_communication_real_time_loop_helper(void *context)
  {
    return static_cast<DynamicGraphManager *>(context)->
        hardware_communication_real_time_loop();
  }

  /**
   * @brief single_process_real_time_loop is the method that performs the
   * control but in one single process. (torque, position, current, ...)
   */
  void* single_process_real_time_loop();

  /**
   * @brief dynamic_graph_real_time_loop_helper is a static member allowing to
   * use the posix pthread_create.
   * @param context is the DynamicGraphManager that spawned the thread.
   * @return nothing interesting for us.
   */
  static void* single_process_real_time_loop_helper(void *context)
  {
    return static_cast<DynamicGraphManager *>(context)->
        single_process_real_time_loop();
  }

  /***********************
   *  Private attributes *
   ***********************/
protected:
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
  std::unique_ptr<real_time_tools::RealTimeThread> thread_dynamic_graph_;

  /**
   * @brief thread_hardware_communication_ is the real thread that communicate
   * with the hardware.
   */
  std::unique_ptr<real_time_tools::RealTimeThread> thread_hardware_communication_;

  /**
   * @brief pid_dynamic_graph_process_ is the pid of the DynamicGraph process.
   * It is initialized to 0 and set during the DynamicGraphManager::run method
   */
  pid_t pid_dynamic_graph_process_;

  /**
   * @brief pid_hardware_communication_process_ is the pid of the hardware
   * communication process. It is initialized to 0 and set during the
   * DynamicGraphManager::run method
   */
  pid_t pid_hardware_communication_process_;

  /**
   * @brief params_ is the pool of parameters in a yaml tree
   */
  YAML::Node params_;

  /**
   * @brief device_ is the DynamicGraph device that manages the computation of
   * the graph.
   */
  std::unique_ptr<Device> device_;

  /**
    * @brief sensors_map_ is a map of dynamicgraph::Vector. They represent
    * all the sensors data measured on the robot.
    */
  VectorDGMap sensors_map_ ;

  /**
   * @brief motor_controls_map_ is a map of dynamicgraph::Vector. They represent
    * all the controls to be sent to the robot.
   */
  VectorDGMap motor_controls_map_ ;

  /**
   * @brief cond_var_sensors_ this condition variable allow the computation of
   * the dynamic graph just after the acquisition of the sensors
   */
  std::unique_ptr<shared_memory::ConditionVariable> cond_var_;

  /**
   * @brief has_been_waken_by_dg_ is a flag that indicates if the hardware
   * communication process has been awaken by the dynamic_graph process or not.
   */
  bool has_been_waken_by_dg_;

  /**
   * @brief missed_control_count_ is counting the number of iteration when the
   * dynamic_graph failed to provide data.
   */
  unsigned missed_control_count_;

  /**
   * @brief max_missed_control_ if the missed_control_count_ reach the value of
   * max_missed_control_ then we switch to safety mode.
   */
  unsigned max_missed_control_;

  /**
   * @brief control_period_sec_ this is the control period in Seconds
   * (S.I. units) for computation.
   */
  double control_period_sec_ ;

  /**
   * @brief control_period_ this is the control period in nanoseconds.
   */
  clock::duration control_period_ ;

  /**
   * @brief hw_time_loop_before_sleep_ is the time measurement just before the
   * hardware communication loop goes to sleep.
   */
  clock::time_point hw_time_loop_before_sleep_;

  /**
   * @brief hw_time_loop_after_sleep_ is the time measurement just after the
   * hardware communication loop goes to sleep.
   */
  clock::time_point hw_time_loop_after_sleep_;

  /**
   * @brief hw_measured_sleep_time_ is the time during which the hardware
   * communication process actually slept.
   */
  clock::duration hw_meas_sleep_time_;

  /**
   * @brief hw_ref_sleep_time_ is the time during which the hardware
   * communication process is supposed to sleep.
   */
  clock::duration hw_ref_sleep_time_;

  /**
   * @brief hw_meas_active_time_ is the time during which the hardware
   * communication process is supposed to sleep.
   */
  clock::duration hw_meas_active_time_;

  /**
   * @brief is_real_robot this boolean is a parameter to indicate if yes or no
   * we are in simulation or in a real robot mode.
   */
  bool is_real_robot_;

  /**
   * @brief dg_timer_file_ this is the path to the file that will contain the
   * computation time of each of the dynamic graph complete execution.
   */
  std::string dg_timer_file_;

  /**
   * @brief hwc_active_timer_file_ this is the path to the file that will
   * contain the computation time of each active period of the hardware
   * communication loop.
   */
  std::string hwc_active_timer_file_;

  /**
   * @brief hwc_sleep_timer_file_ this is the path to the file that will
   * contain the sleeping time of the hardware communication loop.
   */
  std::string hwc_sleep_timer_file_;

  /**
   * @brief hwc_timer_file_ this is the path to the file that will contain the
   * computation time of each of the hardware communication complete execution.
   */
  std::string hwc_timer_file_;

  /**
   * @brief log_folder_ is the folder where all the data of the current
   * experiment will be saved.
   */
  std::string log_dir_;

  /**
   * @brief This file will contain the python interpreter output.
   */
  std::string python_log_file_;

  /**
   * @brief This is the application directory in the home directory.
   */
  std::string app_dir_;

  /**
   * @brief dg_timer is the timer for the dynamic graph loop
   */
  real_time_tools::Timer dg_timer_;

  /**
   * @brief hwc_active_timer is measuring the active time of the hardware
   * communication loop
   */
  real_time_tools::Timer hwc_active_timer_;

  /**
   * @brief hwc_sleep_timer is measuring the sleeping time of the hardware
   * communication loop
   */
  real_time_tools::Timer hwc_sleep_timer_;

  /**
   * @brief hwc_timer is measuring the time of the hardware communication loop
   */
  real_time_tools::Timer hwc_timer_;

  /**
   * @brief memory_buffer_timers_ is the size of the memory buffers for the
   * real_time_tools timers.
   */
  unsigned memory_buffer_timers_;

  /**
   * @brief This class allows us to time the real time thread for the hardware
   * communication.
   */
  real_time_tools::Spinner hwc_spinner_;

  /**
   * @brief This corresponds to the predicted sleeping time for the hardware
   * communication process. If this time is bigger than a certain threshold
   * then user commands to the hardware can be sent.
   */
  double hwc_predicted_sleeping_time_;

  /**
   * @brief This is the list of the user commands.
   */
  std::deque<std::function<void(void)> > user_commands_;
};

} // namespace dynamic_graph

#endif // DYNAMIC_GRAPH_MANAGER_HH
