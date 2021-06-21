/**
 * @file dynamic_graph_manager.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

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
 *   - [1] Ask the drivers to initialize the communication with the hardware
 *   - [2] Loads a yaml/urdf config file.
 *   - [3] Spawn the first real time process that executes the following:
 *      - [3.1] gets  the  sensor  data  using Drivers and  saves  them  in  the
 *              shared std::map sensors
 *      - [3.2] reads the control values in the shared std::map commands and
 *              send them to the motors via the Drivers
 *
 * In this class we heavily depend on std::unique pointers in order to
 * initialize the DynamicGraph process and the hardware communication process
 * independently.
 */
class HardwareProcess
{
    /*******************
     *  Public methods *
     *******************/
public:
    /**
     * @brief HardwareProcess, constructor of the class
     */
    HardwareProcess();

    /**
     * @brief HardwareProcess, destructor of the class
     */
    virtual ~HardwareProcess();

    /**
     * @brief initialize the basic variables
     */
    void initialize(std::string yaml_file_path);

    /**
     * @brief run() Initializes the hardware_communication drivers, spawns
     * a realtime thread for the communication and starts the thread.
     */
    void run();

    /**
     * @brief wait_stop_hardware_communication put the current thread to sleep
     * until the user stop the hardware communication.
     */
    void wait_stop_hardware_communication();

    /***************************
     * method to be overloaded *
     ***************************/

    /**
     * @brief initialize_drivers instanciate all variables related to the
     * hardware communication. In addition it spawns the real time thread.
     * WARNING, this function needs to be overloaded using
     * the actual drivers of the robot.
     */
    virtual void initialize_drivers() = 0;

    /**
     * @brief get_sensors_to_map is the fonction that get the motor command from
     * a map and that uses the drivers to send these command to the robot.
     * Each robot must have a different implementation of this function.
     * WARNING, this function needs to be overloaded using the actual
     * drivers of the robot.
     */
    virtual void get_sensors_to_map(VectorDGMap&) = 0;

    /**
     * @brief set_motor_controls_from_map is the fonction that get the motor
     * command from a map and that uses the drivers to send these command to the
     * robot. Each robot must have a different implementation of this function.
     * WARNING, this function needs to be overloaded using the actual
     * drivers of the robot.
     */
    virtual void set_motor_controls_from_map(const VectorDGMap&) = 0;

    /**
     * @brief is_in_safety_mode check if the dynamic graph is still alive and
     * sending commands at a descent frequency.
     * @return true if there is a problem
     *
     * @todo implement heart beat check.
     */
    virtual bool is_in_safety_mode() = 0;

    /**
     * @brief compute_safety_controls computes safety controls very fast in case
     * the dynamic graph is taking to much computation time or has crashed.
     */
    virtual void compute_safety_controls() = 0;

    /************************
     * getters and setters *
     ************************/

    /**
     * @brief stop_hardware_communication stops the hardware communication. ;)
     */
    void stop()
    {
        is_hardware_communication_stopped_ = true;
    }

    /**
     * @brief start_hardware_communication starts the hardware communication. ;)
     */
    void start()
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
     * @brief has_dynamic_graph_process_died check if the process of the
     * DynamicGraph has died or not.
     * @return true if the DynamicGraph process died.
     */
    bool is_control_process_alive();

    /**
     * @brief com_ros_node_name_ this is the ros node name of the harware
     * communication process
     */
    std::string com_ros_node_name_;

    /**
     * @brief shared_memory_name is the name of the shared memory segment to be
     * used
     */
    std::string shared_memory_name_;

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
    std::string cond_var_name_;

    /**
     * Method inherited
     */
protected:
    /**
     * @brief This method allow to simply add a user command
     */
    void add_user_command(std::function<void(void)> func);

private:
    /**
     * @brief hardware_communication_real_time_loop is the method that
     * communicate with the hardware and send the commands (torque, position,
     * current, ...)
     */
    void* hardware_communication_real_time_loop();

    /**
     * @brief dynamic_graph_real_time_loop_helper is a static member allowing to
     * use the posix pthread_create.
     * @param context is the HardwareProcess that spawned the thread.
     * @return nothing interesting for us.
     */
    static void* hardware_communication_real_time_loop_helper(void* context)
    {
        return static_cast<HardwareProcess*>(context)
            ->hardware_communication_real_time_loop();
    }

    /***********************
     *  Private attributes *
     ***********************/
protected:
    /**
     * @brief is_hardware_communication_stopped_ is the flag reflecting the
     * state of the hardware communication thread.
     *  - TRUE the hardware communication is NOT running.
     *  - FALSE the hardware communication IS running.
     * The type "atomic" is here to make sure that this variable is thread safe
     */
    std::atomic<bool> is_hardware_communication_stopped_;

    /**
     * @brief thread_hardware_communication_ is the real thread that communicate
     * with the hardware.
     */
    std::unique_ptr<real_time_tools::RealTimeThread>
        thread_hardware_communication_;

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
     * @brief has_been_waken_by_dg_ is a flag that indicates if the hardware
     * communication process has been awaken by the dynamic_graph process or
     * not.
     */
    bool has_been_waken_by_ctrl_;

    /**
     * @brief missed_control_count_ is counting the number of iteration when the
     * dynamic_graph failed to provide data.
     */
    unsigned missed_control_count_;

    /**
     * @brief max_missed_control_ if the missed_control_count_ reach the value
     * of max_missed_control_ then we switch to safety mode.
     */
    unsigned max_missed_control_;

    /**
     * @brief control_period_ this is the control period in nanoseconds.
     */
    clock::duration control_period_;

    /**
     * @brief time_loop_before_sleep_ is the time measurement just before the
     * hardware communication loop goes to sleep.
     */
    clock::time_point time_loop_before_sleep_;

    /**
     * @brief time_loop_after_sleep_ is the time measurement just after the
     * hardware communication loop goes to sleep.
     */
    clock::time_point time_loop_after_sleep_;

    /**
     * @brief measured_sleep_time_ is the time during which the hardware
     * communication process actually slept.
     */
    clock::duration meas_sleep_time_;

    /**
     * @brief ref_sleep_time_ is the time during which the hardware
     * communication process is supposed to sleep.
     */
    clock::duration ref_sleep_time_;

    /**
     * @brief meas_active_time_ is the time during which the hardware
     * communication process is supposed to sleep.
     */
    clock::duration meas_active_time_;

    /**
     * @brief active_timer_file_ this is the path to the file that will
     * contain the computation time of each active period of the hardware
     * communication loop.
     */
    std::string active_timer_file_;

    /**
     * @brief sleep_timer_file_ this is the path to the file that will
     * contain the sleeping time of the hardware communication loop.
     */
    std::string sleep_timer_file_;

    /**
     * @brief timer_file_ this is the path to the file that will contain the
     * computation time of each of the hardware communication complete
     * execution.
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
     * @brief active_timer is measuring the active time of the hardware
     * communication loop
     */
    real_time_tools::Timer active_timer_;

    /**
     * @brief sleep_timer is measuring the sleeping time of the hardware
     * communication loop
     */
    real_time_tools::Timer sleep_timer_;

    /**
     * @brief timer is measuring the time of the hardware communication loop
     */
    real_time_tools::Timer timer_;

    /**
     * @brief memory_buffer_timers_ is the size of the memory buffers for the
     * real_time_tools timers.
     */
    unsigned memory_buffer_timers_;

    /**
     * @brief This class allows us to time the real time thread for the hardware
     * communication.
     */
    real_time_tools::FrequencyManager spinner_;

    /**
     * @brief This corresponds to the predicted sleeping time for the hardware
     * communication process. If this time is bigger than a certain threshold
     * then user commands to the hardware can be sent.
     */
    double predicted_sleeping_time_;

    /**
     * @brief This the duration during which a user command can be executed.
     */
    double maximum_time_for_user_cmd_;

    /**
     * @brief This is the list of the user commands.
     */
    std::deque<std::function<void(void)> > user_commands_;

    /**
     * Attribute shared with the daughter class
     */
protected:
    /**
     * @brief This is the list of the ros user commands. The class inheriting
     * from this one can add services for the hardware communication process.
     */
    std::vector<ServiceBasePtr> ros_user_commands_;

    /**
     * @brief control_period_sec_ this is the control period in Seconds
     * (S.I. units) for computation.
     */
    double control_period_sec_;

    /**
     * @brief params_ is the pool of parameters in a yaml tree
     */
    YAML::Node params_;

    /**
     * @brief User command mutex.
     */
    std::mutex user_cmd_mutex_;

};

}  // namespace dynamic_graph_manager
