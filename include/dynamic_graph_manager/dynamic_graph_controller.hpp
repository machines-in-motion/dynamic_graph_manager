/**
 * @file dynamic_graph_manager.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef DYNAMIC_GRAPH_MANAGER_HH
#define DYNAMIC_GRAPH_MANAGER_HH

// used to join the different processes
#include <unistd.h>
#ifndef __APPLE__
#include <wait.h>
#endif  // __APPLE__

// use of std::bind
#include <functional>

// used to synchronise the control loop
#include <chrono>

// here this is used to use atomic (thread safe objects)
#include <atomic>

// use to protect the add and consumption of the user commands
#include <mutex>

// get the yaml configuration
#include "yaml_utils/yaml_cpp_fwd.hpp"

// used to spawn the real time thread
#include "real_time_tools/thread.hpp"

// use the realtime spinner to time the loops
#include <real_time_tools/spinner.hpp>
// cadence the single process.
#include <real_time_tools/frequency_manager.hpp>

// time measurement
#include "real_time_tools/timer.hpp"

// used to deal with shared memory
#include "shared_memory/locked_condition_variable.hpp"

// import the python interpreter ros binding
#include "dynamic_graph_manager/ros_python_interpreter_server.hpp"

// some useful tools like the yaml parsing
#include "dynamic_graph_manager/tools.hpp"

// the device of the dynamic-graph
#include "dynamic_graph_manager/device.hpp"

namespace dynamic_graph_manager
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
class DynamicGraphController
{
    /*******************
     *  Public methods *
     *******************/
public:
    /**
     * @brief DynamicGraphController, constructor of the class
     */
    DynamicGraphController();

    /**
     * @brief DynamicGraphController, destructor of the class
     */
    virtual ~DynamicGraphController();

    /**
     * @brief initialize the basic variables
     */
    void initialize(std::string yaml_file_path);

    /**
     * @brief run() splits the process in the dynamic_graph process and the
     * hadware_communication process. It initialize them and run them. WARNING
     * this a NONE blocking function. One can spin endlessly using the ROS:
     * ros::waitForShutdown(), for example.
     */
    void run();

    /**
     * @brief wait_start put the current thread to sleep until the
     * user start the dynamic graph
     */
    void wait_start();

    /**
     * @brief wait_stop put the current thread to sleep until the
     * user stop the dynamic graph
     */
    void wait_stop();

    /**
     * @brief run_python_command
     * @param file is the logging file to log the entry
     * @param command is the python command itself
     */
    void run_python_command(std::ostream& file, const std::string& command);

    /**
     * @brief python_prologue get the pointer of the device in the the python
     * interpretor.
     */
    void python_prologue();

    /************************
     * getters and setters *
     ************************/

    /**
     * @brief stop stop the DynamicGraph. ;)
     */
    void stop()
    {
        is_stopped_ = true;
    }

    /**
     * @brief start start the DynamicGraph. ;)
     */
    void start()
    {
        is_stopped_ = false;
    }

    /**
     * @brief get the status of the dynamic graph (is running or not)
     * @return the flag is_stopped_ value
     */
    bool is_stopped()
    {
        return is_stopped_;
    }

    /**
     * @brief device is a getter method on the Device internal pointer.
     * @return a const reference to the device.
     */
    Device& device()
    {
        if (device_ != nullptr) return *device_;
        throw(
            std::runtime_error("DynamicGraphController::device():Try"
                               "accessing a device that has not been created"));
    }

    /**
     * @brief ros_node_name_ this is the ros node name of the dynamic graph
     * process
     */
    static const std::string ros_node_name_;

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
     * @brief motor_controls_map_name is the name of the motor controls map
     * inside the shared memory segment
     */
    static const std::string motor_controls_map_name_;

    /**
     * @brief cond_var_sensors_name_ is the name of the condition variable in
     * the shared memory
     */
    static const std::string cond_var_name_;

    /**
     * Method NOT inherited
     */
private:
    /**
     * @brief start_dg is the callback method of the ROS service start dynamic
     * graph.
     */
    void start(std_srvs::srv::Empty::Request::SharedPtr,
                             std_srvs::srv::Empty::Response::SharedPtr)
    {
        start();
    }

    /**
     * @brief stop_dg is the callback method of the ROS service stop dynamic
     * graph
     */
    void stop(std_srvs::srv::Empty::Request::SharedPtr,
                            std_srvs::srv::Empty::Response::SharedPtr)
    {
        stop();
    }

public:
    /**
     * @brief start_ros_service is the method that advertise the different ros
     * services.
     */
    void start_ros_service();

private:
    /**
     * @brief dynamic_graph_real_time_loop is the method used to execute the
     * dynamic graph.
     */
    void* dynamic_graph_real_time_loop();

    /**
     * @brief dynamic_graph_real_time_loop_helper is a static member allowing to
     * use the posix pthread_create.
     * @param context is the DynamicGraphController that spawned the thread.
     * @return nothing interesting for us.
     */
    static void* dynamic_graph_real_time_loop_helper(void* context)
    {
        return static_cast<DynamicGraphController*>(context)
            ->dynamic_graph_real_time_loop();
    }

    /***********************
     *  Private attributes *
     ***********************/
protected:
    /**
     * @brief ros_service_start_ allows to start the dynamic graph on call.
     * It simply sets a flags that is used to wait the user call. Only used in
     * the dynamic_graph process.
     */
    EmptyServicePtr ros_service_start_;

    /**
     * @brief ros_service_stop_ allows to stop the dynamic graph on call.
     * It simply sets a flags that stop the main real time the control loop.
     * Only used in the dynamic_graph process.
     */
    EmptyServicePtr ros_service_stop_;

    /**
     * @brief is_stopped_ is the flag reflecting the state of the
     * dynamic graph.
     *  - TRUE the Dynamic Graph is NOT running.
     *  - FALSE the Dynamic Graph IS running.
     * The type "atomic" is here to make sure that this variable is thread safe
     */
    std::atomic<bool> is_stopped_;

    /**
     * @brief ros_python_interpreter_ptr_ is a ROS wrapper around a python
     * interpreter.
     */
    std::unique_ptr<dynamic_graph_manager::RosPythonInterpreterServer>
        ros_python_interpreter_;

    /**
     * @brief thread_ is the real time thread that runs the
     * dynamic graph.
     */
    std::unique_ptr<real_time_tools::RealTimeThread> rt_thread_;

    /**
     * @brief device_ is the DynamicGraph device that manages the computation of
     * the graph.
     */
    std::unique_ptr<Device> device_;

    /**
     * @brief sensors_map_ is a map of dynamicgraph::Vector. They represent
     * all the sensors data measured on the robot.
     */
    VectorDGMap sensors_map_;

    /**
     * @brief motor_controls_map_ is a map of dynamicgraph::Vector. They
     * represent all the controls to be sent to the robot.
     */
    VectorDGMap motor_controls_map_;

    /**
     * @brief cond_var_sensors_ this condition variable allow the computation of
     * the dynamic graph just after the acquisition of the sensors
     */
    std::unique_ptr<shared_memory::LockedConditionVariable> cond_var_;

    /**
     * @brief control_period_ this is the control period in nanoseconds.
     */
    clock::duration control_period_;

    /**
     * @brief active_timer_file_ this is the path to the file that will
     * contain the computation time of each of the dynamic graph complete
     * execution.
     */
    std::string active_timer_file_;

    /**
     * @brief sleep_timer_file_ this is the path to the file that will
     * contain the sleep duration of the dynamic graph thread.
     */
    std::string sleep_timer_file_;

    /**
     * @brief timer_file_ this is the path to the file that will contain the
     * time of the dynamic graph loop.
     */
    std::string timer_file_;

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
     * @brief active_timer_ is the timer measuring the computation time of
     * the dynamic graph loop.
     */
    real_time_tools::Timer active_timer_;

    /**
     * @brief sleep_timer_ is the timer measuring the time during which
     * the dynamic graph loop sleeps.
     */
    real_time_tools::Timer sleep_timer_;

    /**
     * @brief timer_ is the timer measuring the duration time of the dynamic
     * graph loop.
     */
    real_time_tools::Timer timer_;

    /**
     * @brief memory_buffer_timers_ is the size of the memory buffers for the
     * real_time_tools timers.
     */
    unsigned memory_buffer_timers_;

    /**
     * Attribute shared with the daughter class
     */
protected:
    /**
     * @brief control_period_sec_ this is the control period in Seconds
     * (S.I. units) for computation.
     */
    double control_period_sec_;

    /**
     * @brief params_ is the pool of parameters in a yaml tree
     */
    YAML::Node params_;
};

}  // namespace dynamic_graph_manager

#endif  // DYNAMIC_GRAPH_MANAGER_HH
