/**
 * @file dynamic_graph_manager.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

// use for real_time printf
#include <real_time_tools/iostream.hpp>
// use to set cpu latency.
#include <real_time_tools/process_manager.hpp>
// use the ROS singleton to initialize and use ROS
#include <dynamic_graph_manager/ros.hpp>
// this file defines the class in this header
#include <dynamic_graph_manager/hardware_process.hpp>
// in order to throw hand made exception
#include <dynamic_graph_manager/exception/exception-yaml-cpp.hpp>

#define DYNAMIC_GRAPH_MANAGER_VERBOSE 0

using namespace dynamic_graph_manager;

const std::string HardwareProcess::sensors_map_name_ = "sensors_map";
const std::string HardwareProcess::motor_controls_map_name_ =
    "motor_controls_map";

HardwareProcess::HardwareProcess()
{
    params_.reset();

    sensors_map_.clear();
    motor_controls_map_.clear();

    has_been_waken_by_ctrl_ = false;

    missed_control_count_ = 0;
    max_missed_control_ = 0;
    control_period_ = clock::duration(0);

    // files where to dump the timers
    log_dir_ = "/tmp/";
    active_timer_file_ = "/tmp/active_timer.dat";
    sleep_timer_file_ = "/tmp/sleep_timer.dat";
    timer_file_ = "/tmp/timer.dat";

    once_safe_mode_msg_ = false;
}

HardwareProcess::~HardwareProcess()
{
    // kill all ros related stuff
    ros_shutdown();

    // wait for the hardware communication thread to stop
    stop();
    wait_stop_hardware_communication();

    // clean the shared memory
    shared_memory::clear_shared_memory(shared_memory_name_);
}

void HardwareProcess::initialize(std::string yaml_file_path)
{
    YAML::Node param = YAML::LoadFile(yaml_file_path);

    // copy the yaml node for further use
    params_ = param;

    YAML::ReadParameter(
        params_["hardware_communication"], "shared_memory_name", shared_memory_name_);
    cond_var_name_ = shared_memory_name_ + "_cond_var";

    YAML::ReadParameter(
        params_["hardware_communication"], "ros_node_name", com_ros_node_name_);

    shared_memory::clear_shared_memory("dgm_shm_name");
    shared_memory::set<std::string>(
        "dgm_shm_name", "shared_memory_name", shared_memory_name_);

    // Upon initialization the graph and the hardware com are inactive
    stop();

    // clean the shared memory
    shared_memory::clear_shared_memory(shared_memory_name_);
    // we initialize the sensors and control maps in the common initializer
    parse_yaml_node(params_["device"], sensors_map_, motor_controls_map_);
    shared_memory::set(
        shared_memory_name_, sensors_map_name_, sensors_map_);
    shared_memory::set(
        shared_memory_name_, motor_controls_map_name_, motor_controls_map_);

    // Set the maximum cpu latency to 0 us. This keeps the CPU from sleeping.
    real_time_tools::set_cpu_dma_latency(0);

    // get the parameter for the hardware communication loop
    std::string error_str =
        "Fail to parse yaml file. Node is: "
        "hardware_communication:    ";
    try
    {
        max_missed_control_ =
            params_["hardware_communication"]["max_missed_control"]
                .as<unsigned>();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        throw ExceptionYamlCpp(ExceptionYamlCpp::PARSING_UNSIGNED,
                               error_str + "max_missed_control");
    }
    try
    {
        control_period_ = clock::duration(
            params_["hardware_communication"]["control_period"].as<unsigned>());
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        throw ExceptionYamlCpp(ExceptionYamlCpp::PARSING_UNSIGNED,
                               error_str + "control_period");
    }
    try
    {
        control_period_sec_ =
            params_["hardware_communication"]["control_period"].as<double>() *
            std::pow(10, -9);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        throw ExceptionYamlCpp(ExceptionYamlCpp::PARSING_DOUBLE,
                               error_str + "control_period");
    }
    try
    {
        maximum_time_for_user_cmd_ =
            params_["hardware_communication"]["maximum_time_for_user_cmd"]
                .as<double>() *
            std::pow(10, -9);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        throw ExceptionYamlCpp(ExceptionYamlCpp::PARSING_DOUBLE,
                               error_str + "maximum_time_for_user_cmd");
    }

    log_dir_ = real_time_tools::get_log_dir("dynamic_graph_manager");
    active_timer_file_ = log_dir_ + "active_timer.dat";
    sleep_timer_file_ = log_dir_ + "sleep_timer.dat";
    timer_file_ = log_dir_ + "timer.dat";

    unsigned int debug_timer_history_length = 0;
    YAML::ReadParameter(params_,
                        "debug_timer_history_length",
                        debug_timer_history_length,
                        true);

    active_timer_.set_memory_size(debug_timer_history_length);
    sleep_timer_.set_memory_size(debug_timer_history_length);
    timer_.set_memory_size(debug_timer_history_length);

    std::cout << "HARDWARE: Timer logs will be saved in: \"" << log_dir_ << "\"" << std::endl;

    // we create and destroy the condition variable to free the shared memory
    // and therefore the associated mutex which must be lockable at this state.
    {
        shared_memory::LockedConditionVariable(cond_var_name_, true);
    }
}

void HardwareProcess::run()
{
    pid_t pid_hardware_communication_process_ = getpid();
    std::cout << "pid of hardware communication process: "
                << pid_hardware_communication_process_ << std::endl;

    initialize_drivers();

    cond_var_.reset(
        new shared_memory::LockedConditionVariable(cond_var_name_, true));

    // Allow the hardware thread to run.
    start();

    int cpu_id;
    YAML::ReadParameter(
        params_["hardware_communication"], "cpu_id", cpu_id);

    // Launch the real time thread.
    thread_hardware_communication_.reset(new real_time_tools::RealTimeThread());
    thread_hardware_communication_->parameters_.cpu_id_.push_back(cpu_id);
    thread_hardware_communication_->create_realtime_thread(
        &HardwareProcess::hardware_communication_real_time_loop_helper,
        this);
    printf("HARDWARE: communication loop started\n");

    // From here on this process is a ros node.
    get_ros_node(com_ros_node_name_);
    ros_add_node_to_executor(com_ros_node_name_);

    ros_spin_non_blocking();
}

void HardwareProcess::wait_stop_hardware_communication()
{
    while (!is_hardware_communication_stopped())
    {
        real_time_tools::Timer::sleep_sec(0.1);
    }
    stop();
    if (thread_hardware_communication_)
    {
        thread_hardware_communication_->join();
    }
}

void* HardwareProcess::hardware_communication_real_time_loop()
{
    // We acquiere the lock on the condition variable here.
    cond_var_->lock_scope();

    // Some basic checks.
    assert(!is_hardware_communication_stopped_ && "The loop is started");
    get_ros_node(com_ros_node_name_);
    assert(ros_ok() && "Ros has to be initialized");

    // Initialize the motor_controls_map with zeros.
    for (VectorDGMap::iterator ctrl = motor_controls_map_.begin();
         ctrl != motor_controls_map_.end();
         ++ctrl)
    {
        ctrl->second.fill(0.0);
    }

    // Initialize the time measurements
    active_timer_.tic();
    timer_.tic();

    // time the loop so it respect the input frequency/period
    spinner_.set_period(control_period_sec_);

    // we start the main loop
    rt_printf("HARDWARE: Start main realtime control loop.\n");
    while (!is_hardware_communication_stopped() && ros_ok())
    {
        // rt_printf("HARDWARE: Call the sensors. \n");
        if (!is_hardware_communication_stopped() && ros_ok())
        {
            get_sensors_to_map(sensors_map_);
        }

        // rt_printf("HARDWARE: Write sensors in the shared memory. \n");
        if (cond_var_->owns() || cond_var_->try_lock())
        {
            // write the sensors to the shared memory
            shared_memory::set(
                shared_memory_name_, sensors_map_name_, sensors_map_);

            // Unlock the mutex such that the task process can acquire it right
            // away after calling `notify_all()`.
            cond_var_->unlock();

            // Notify the control process that it can compute the graph.
            // This notify_all also acts as time-synchronization between the
            // task and motor process.
            cond_var_->notify_all();
        }

        // predict here the sleeping time in the spinner
        predicted_sleeping_time_ = spinner_.predict_sleeping_time();
        if (predicted_sleeping_time_ > maximum_time_for_user_cmd_)
        {
            if (user_commands_.size() > 0)
            {
                user_cmd_mutex_.lock();
                user_commands_[0]();
                user_commands_.pop_front();
                user_cmd_mutex_.unlock();
                rt_printf("HARDWARE: Executed user command. \n");
            }
        }

        // here is the end of the thread activity
        active_timer_.tac();
        timer_.tac_tic();

        // Sleeps for one period. This clocks the motor process.
        // rt_printf("HARDWARE: Sleep. \n");
        sleep_timer_.tic();
        spinner_.wait();
        sleep_timer_.tac();

        // here is the beginning of the thread activity
        active_timer_.tic();

        // TODO: Check if realtime was lost here. Using realtime_check always
        // recorded switches for jviereck.

        // If the control process finished its computation, it is waiting
        // and we are able to acquire the lock here.
        if (cond_var_->try_lock())
        {
            // A new control command is available.
            missed_control_count_ = 0;
        }
        else
        {
            ++missed_control_count_;
        }

        if (is_in_safety_mode())
        {
            if (!once_safe_mode_msg_)
            {
                rt_printf("HARDWARE: Warning enter into safe_mode\n");
                once_safe_mode_msg_ = true;
            }
            compute_safety_controls();
            // we write the safety command in the shared memory in order to
            // perform some potential interpolation in the control graph
            shared_memory::set(shared_memory_name_,
                               motor_controls_map_name_,
                               motor_controls_map_);
        }
        else if (missed_control_count_ == 0)
        {
            // If the wait on the conditional variable was successful, then read
            // the values from shared memory.
            shared_memory::get(shared_memory_name_,
                               motor_controls_map_name_,
                               motor_controls_map_);
        }

        // we do not send the command if the thread is asked to stopped
        if (!is_hardware_communication_stopped() && ros_ok())
        {
            // send the command to the motors
            set_motor_controls_from_map(motor_controls_map_);
        }
    }
    // We use this function here because the loop might stop because of ROS
    // and we need the flag to be set to off
    stop();

    rt_printf("HARDWARE: Dump active time measurement \n");
    active_timer_.dump_measurements(active_timer_file_);
    rt_printf("HARDWARE: sleep time measurement \n");
    sleep_timer_.dump_measurements(sleep_timer_file_);
    rt_printf("HARDWARE: hwc time measurement \n");
    timer_.dump_measurements(timer_file_);
    cond_var_->unlock_scope();
    rt_printf("HARDWARE: Stop loop \n");

    return THREAD_FUNCTION_RETURN_VALUE;
}

bool HardwareProcess::is_in_safety_mode()
{
    bool too_much_missed_control =
        (missed_control_count_ >= max_missed_control_);
    if (too_much_missed_control && !once_too_much_missed_control_msg_)
    {
        rt_printf(
            "HARDWARE: Too much missed control (%d/%d), going in safe "
            "mode\n",
            missed_control_count_,
            max_missed_control_);
        once_too_much_missed_control_msg_ = true;
    }

    // TODO: Check for heart beat here.
    // bool dg_died = has_dynamic_graph_process_died();
    // static bool once_hdgd = false;
    // if (dg_died && !once_hdgd)
    // {
    //     rt_printf(
    //         "HARDWARE: Dynamic Graph process died, going in safe mode\n");
    //     once_hdgd = true;
    // }
    return too_much_missed_control; // || dg_died;
}

void HardwareProcess::add_user_command(std::function<void(void)> func)
{
    user_cmd_mutex_.lock();
    user_commands_.push_back(func);
    user_cmd_mutex_.unlock();
}
