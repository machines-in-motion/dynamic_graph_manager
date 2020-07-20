/**
 * @file dynamic_graph_manager.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22 */

#pragma once

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

// ROS includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// used to spawn the real time thread
#include "real_time_tools/thread.hpp"

// use the realtime spinner to time the loops
#include "real_time_tools/spinner.hpp"

// time measurement
#include "real_time_tools/timer.hpp"

// manage the shared memory naming
#include "dynamic_graph_manager/shared_memory.hpp"

// import the python interpreter ros binding
#include "dynamic_graph_manager/ros_interpreter.hpp"

// some useful tools like the yaml parsing
#include "dynamic_graph_manager/yaml_parsing.hpp"

// the device of the dynamic-graph
#include "dynamic_graph_manager/device.hpp"

namespace dynamic_graph_manager
{
/** @brief Controller based on the dynamic graph.
 *
 * The main tasks of this class are:
 *   - [1] Creates the main objects:
 *       - [1.1] the dynamic-graph device entity,
 *       - [1.2] the python interpreter
 *   - [2] Ask the python interpreter to advertise its ROS services
 *   - [3] Loads a yaml/urdf config file.
 *   - [4] Advertise the ROS services start/stop dynamic graph
 *   - [5] Wait for the ROS service start dynamic graph to be called
 *   - [6] Spawn the real time thread that executes the following:
 *      - [6.1] passes the std::map sensors to the Device, which copies the data
 *              to its output signals
 *      - [6.2] gets the control values from the Device (which triggers the
 *              evaluation of  the  dynamic  graph)  and  copies  them  into
 *              the shared std::map commands. */
class DynamicGraphController
{
    /*******************
     *  Public methods *
     *******************/
public:
    /** @brief DynamicGraphController, constructor of the class */
    DynamicGraphController();

    /** @brief DynamicGraphController, destructor of the class */
    virtual ~DynamicGraphController();

    /** @brief initialize the basic variables */
    void initialize(YAML::Node param);

    /** @brief run() initializes the controller and run it. WARNING
     * this a NONE blocking function. One can spin endlessly using the ROS
     * `ros::waitForShutdown()` after calling this method, for example. */
    void run();

    /** @brief wait_until_controller_starts put the current thread to sleep
     * until the user start the controller */
    void wait_until_controller_starts();

    /** @brief wait_until_controller_stops put the current thread to sleep until
     * the user stop the dynamic graph */
    void wait_until_controller_stops();

    /** @brief run_python_command
     * @param file is the logging file to log the entry
     * @param command is the python command itself */
    void run_python_command(std::ostream& file, const std::string& command);

    /** @brief stop stop the DynamicGraph. ;) */
    void stop()
    {
        is_stopped_ = true;
    }

    /** @brief start start the DynamicGraph. ;) */
    void start()
    {
        is_stopped_ = false;
    }

    /** @brief get the status of the dynamic graph (is running or not)
     * @return the flag is_stopped_ value */
    bool is_stopped()
    {
        return is_stopped_;
    }

    /** @brief device is a getter method on the Device internal pointer.
     * @return a const reference to the device. */
    Device& device()
    {
        if (device_ != nullptr) return *device_;
        throw(
            std::runtime_error("DynamicGraphController::device():Try"
                               "accessing a device that has not been created"));
    }

    /** @brief ros_node_name_ this is the ros node name of the dynamic graph
     * process. */
    static const std::string ros_node_name_;

    /** Method inherited */
protected:
    /** Method NOT inherited */
private:
    /** @brief python_prologue get the pointer of the device in the the python
     * interpretor. */
    void python_prologue();

    /** @brief start_dg is the callback method of the ROS service start dynamic
     * graph.
     * @return true.*/
    bool start(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        start();
        return true;
    }

    /** @brief stop_dg is the callback method of the ROS service stop dynamic
     * graph
     * @return */
    bool stop(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        if(cond_var_){
            cond_var_->notify_all();
        }
        stop();
        return true;
    }

    /** @brief advertise_rosservice is the method that advertise the different
     * ros services. */
    void advertise_rosservice();

    /** @brief controller_real_time_loop is the method used to execute the
     * dynamic graph controller. */
    void controller_real_time_loop();

    /** @brief real_time_loop_helper is a static member allowing to
     * use the posix pthread_create.
     * @param context is the DynamicGraphController that spawned the thread.
     * @return nothing interesting for us. */
    static THREAD_FUNCTION_RETURN_TYPE controller_real_time_loop_helper(
        void* context)
    {
        static_cast<DynamicGraphController*>(context)
            ->controller_real_time_loop();
        return THREAD_FUNCTION_RETURN_VALUE;
    }

    /** Private attributes */
protected:

    ros::NodeHandle& ros_node_;

    /** @brief rosservice_start_ allows to start the dynamic graph on call.
     * It simply sets a flags that is used to wait the user call. Only used in
     * the controller process. */
    ros::ServiceServer rosservice_start_;

    /** @brief rosservice_stop_ allows to stop the dynamic graph on call.
     * It simply sets a flags that stop the main real time the control loop.
     * Only used in the controller process. */
    ros::ServiceServer rosservice_stop_;

    /** @brief is_stopped_ is the flag reflecting the state of the
     * dynamic graph.
     *  - TRUE the Dynamic Graph is NOT running.
     *  - FALSE the Dynamic Graph IS running.
     * The type "atomic" is here to make sure that this variable is thread safe
     */
    std::atomic<bool> is_stopped_;

    /** @brief ros_python_interpreter_ptr_ is a ROS wrapper around a python
     * interpreter. */
    std::unique_ptr<dynamic_graph_manager::RosPythonInterpreter>
        ros_python_interpreter_;

    /** @brief Real-time thread that runs the dynamic graph controller. */
    std::unique_ptr<real_time_tools::RealTimeThread> rt_thread_;

    /** @brief device_ is the DynamicGraph device that manages the computation
     * of the graph. */
    std::unique_ptr<Device> device_;

    /** @brief sensors_map_ is a map of dynamicgraph::Vector. They represent
     * all the sensors data measured on the robot. */
    VectorDGMap sensors_map_;

    /** @brief joint_controls_map_ is a map of dynamicgraph::Vector. They
     * represent all the controls to be sent to the robot. */
    VectorDGMap joint_controls_map_;

    /** @brief cond_var_sensors_ this condition variable allow the computation
     * of the dynamic graph just after the acquisition of the sensors */
    std::unique_ptr<shared_memory::LockedConditionVariable> cond_var_;

    /** @brief active_timer_file_ this is the path to the file that will
     * contain the computation time of each of the dynamic graph complete
     * execution. */
    std::string active_timer_file_;

    /** @brief sleep_timer_file_ this is the path to the file that will
     * contain the sleep duration of the dynamic graph thread. */
    std::string sleep_timer_file_;

    /** @brief timer_file_ this is the path to the file that will contain the
     * time of the dynamic graph loop. */
    std::string timer_file_;

    /** @brief log_folder_ is the folder where all the data of the current
     * experiment will be saved. */
    std::string log_dir_;

    /** @brief This file will contain the python interpreter output. */
    std::string python_log_file_;

    /** @brief This is the application directory in the home directory. */
    std::string app_dir_;

    /** @brief active_timer_ is the timer measuring the computation time of
     * the dynamic graph loop. */
    real_time_tools::Timer active_timer_;

    /** @brief sleep_timer_ is the timer measuring the time during which
     * the dynamic graph loop sleeps. */
    real_time_tools::Timer sleep_timer_;

    /** @brief timer_ is the timer measuring the duration time of the dynamic
     * graph loop. */
    real_time_tools::Timer timer_;

    /** @brief memory_buffer_timers_ is the size of the memory buffers for the
     * real_time_tools timers. */
    unsigned memory_buffer_timers_;

    /** @brief params_ is the pool of parameters in a yaml tree */
    YAML::Node params_;

    /** @brief check if the destructor has been called. */
    bool destructor_called_;
};

}  // namespace dynamic_graph_manager
