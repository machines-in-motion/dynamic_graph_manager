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
#include <dynamic_graph_manager/dynamic_graph_manager.hpp>
// in order to throw hand made exception
#include <dynamic_graph_manager/exception/exception-yaml-cpp.hpp>

#define DYNAMIC_GRAPH_MANAGER_VERBOSE 0

using namespace dynamic_graph_manager;

const std::string DynamicGraphManager::dg_ros_node_name_ = DG_ROS_NODE_NAME
;
const std::string DynamicGraphManager::hw_com_ros_node_name_ =
    HWC_ROS_NODE_NAME;
const std::string DynamicGraphManager::sensors_map_name_ = "sensors_map";
const std::string DynamicGraphManager::motor_controls_map_name_ =
    "motor_controls_map";
const std::string DynamicGraphManager::shared_memory_name_ = "DGM_ShM";
const std::string DynamicGraphManager::cond_var_name_ = "cond_var";

DynamicGraphManager::DynamicGraphManager()
{
    shared_memory::clear_shared_memory("dgm_shm_name");
    shared_memory::set<std::string>(
        "dgm_shm_name", "shared_memory_name", shared_memory_name_);

    // Upon construction the graph is inactive
    params_.reset();

    stop_dynamic_graph();
    stop_hardware_communication();

    is_real_robot_ = true;

    pid_dynamic_graph_process_ = 0;
    pid_hardware_communication_process_ = 0;

    sensors_map_.clear();
    motor_controls_map_.clear();

    has_been_waken_by_dg_ = false;

    missed_control_count_ = 0;
    max_missed_control_ = 0;
    control_period_ = clock::duration(0);

    // clean the shared memory
    shared_memory::clear_shared_memory(shared_memory_name_);

    // files where to dump the timers
    log_dir_ = "/tmp/";
    dg_active_timer_file_ = "/tmp/dg_active_timer.dat";
    dg_sleep_timer_file_ = "/tmp/dg_sleep_timer.dat";
    dg_timer_file_ = "/tmp/dg_timer.dat";
    hwc_active_timer_file_ = "/tmp/hwc_active_timer.dat";
    hwc_sleep_timer_file_ = "/tmp/hwc_sleep_timer.dat";
    hwc_timer_file_ = "/tmp/hwc_timer.dat";
}

DynamicGraphManager::~DynamicGraphManager()
{
    // kill all ros related stuff
    ros_shutdown();
    // wait for the dynamic graph thread to stop
    stop_dynamic_graph();
    if (cond_var_)
    {
        cond_var_->notify_all();
    }
    wait_stop_dynamic_graph();
    // wait for the hardware communication thread to stop
    stop_hardware_communication();
    if (cond_var_)
    {
        cond_var_->notify_all();
    }
    wait_stop_hardware_communication();
    // clean the shared memory
    shared_memory::clear_shared_memory(shared_memory_name_);
    // destroy the python interpretor after the device
    device_.reset(nullptr);
    ros_python_interpreter_.reset(nullptr);
    if (pid_hardware_communication_process_ == getpid())
    {
        kill(pid_dynamic_graph_process(), SIGKILL);
        while (!has_dynamic_graph_process_died())
        {
            real_time_tools::Timer::sleep_sec(0.1);
        }
    }
}

void DynamicGraphManager::initialize(YAML::Node param)
{
    // Upon initialization the graph and the hardware com are inactive
    stop_dynamic_graph();
    stop_hardware_communication();

    // copy the yaml node for further use
    params_ = param;

    pid_dynamic_graph_process_ = 0;
    pid_hardware_communication_process_ = 0;

    // clean the shared memory
    shared_memory::clear_shared_memory(shared_memory_name_);
    // we initialize the sensors and control maps in the common initializer
    parse_yaml_node(params_["device"], sensors_map_, motor_controls_map_);
    shared_memory::set(shared_memory_name_, sensors_map_name_, sensors_map_);
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
    try
    {
        is_real_robot_ = params_["is_real_robot"].as<bool>();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        throw ExceptionYamlCpp(
            ExceptionYamlCpp::PARSING_BOOL,
            "Fail to parse yaml file. Node is: \nis_real_robot");
    }

    log_dir_ = real_time_tools::get_log_dir("dynamic_graph_manager");
    dg_active_timer_file_ = log_dir_ + "dg_active_timer.dat";
    dg_sleep_timer_file_ = log_dir_ + "dg_sleep_timer.dat";
    dg_timer_file_ = log_dir_ + "dg_timer.dat";
    hwc_active_timer_file_ = log_dir_ + "hwc_active_timer.dat";
    hwc_sleep_timer_file_ = log_dir_ + "hwc_sleep_timer.dat";
    hwc_timer_file_ = log_dir_ + "hwc_timer.dat";

    unsigned int debug_timer_history_length = 0;
    YAML::ReadParameter(params_,
                        "debug_timer_history_length",
                        debug_timer_history_length,
                        true);

    dg_active_timer_.set_memory_size(debug_timer_history_length);
    dg_sleep_timer_.set_memory_size(debug_timer_history_length);
    dg_timer_.set_memory_size(debug_timer_history_length);
    hwc_active_timer_.set_memory_size(debug_timer_history_length);
    hwc_sleep_timer_.set_memory_size(debug_timer_history_length);
    hwc_timer_.set_memory_size(debug_timer_history_length);

    std::cout << "Log will be saved in : \"" << log_dir_ << "\"" << std::endl;

    // we create and destroy the condition variable to free the shared memory
    // and therefore the associated mutex which must be lockable at this state.
    {
        shared_memory::LockedConditionVariable(cond_var_name_, true);
    }
}

void DynamicGraphManager::run()
{
    if (is_real_robot_)
    {
        pid_t child_pid = fork();
        if (child_pid == 0)  // child process
        {
            pid_dynamic_graph_process_ = getpid();
            pid_hardware_communication_process_ = getppid();

            initialize_dynamic_graph_process();
            run_dynamic_graph_process();
            dynamic_graph_manager::ros_spin();
            dynamic_graph_manager::ros_shutdown();
            std::cout << "DG: End of the dynamic graph process." << std::endl;
            exit(0);
        }
        else if (child_pid > 0)  // parent process
        {
            pid_dynamic_graph_process_ = child_pid;
            pid_hardware_communication_process_ = getpid();
            std::cout << "pid of dynamic graph process: "
                      << pid_dynamic_graph_process_ << std::endl;
            std::cout << "pid of hardware communication process: "
                      << pid_hardware_communication_process_ << std::endl;

            initialize_hardware_communication_process();
            run_hardware_communication_process();
        }
        else
        {
            throw(std::runtime_error(
                "DynamicGraphManager::run(): the fork failed"));
        }
    }
    else
    {
        initialize_dynamic_graph_process();
        initialize_hardware_communication_process();
        run_single_process();
    }
}

void DynamicGraphManager::wait_start_dynamic_graph()
{
    while (is_dynamic_graph_stopped() && ros_ok())
    {
        real_time_tools::Timer::sleep_sec(0.1);
    }
}

void DynamicGraphManager::wait_stop_dynamic_graph()
{
    while (!is_dynamic_graph_stopped() && ros_ok())
    {
        real_time_tools::Timer::sleep_sec(0.1);
    }
    stop_dynamic_graph();
    if (thread_dynamic_graph_)
    {
        cond_var_->notify_all();
        thread_dynamic_graph_->join();
    }
}

void DynamicGraphManager::wait_stop_hardware_communication()
{
    while (!is_hardware_communication_stopped())
    {
        real_time_tools::Timer::sleep_sec(0.1);
    }
    stop_hardware_communication();
    if (thread_hardware_communication_)
    {
        cond_var_->notify_all();
        thread_hardware_communication_->join();
    }
}

bool DynamicGraphManager::has_dynamic_graph_process_died()
{
    bool is_dg_proc_dead = false;
    int status = 0;
    pid_t p = 0;
    p = waitpid(pid_dynamic_graph_process_, &status, WNOHANG);
    if (p == 0)
    {
        is_dg_proc_dead = false;
    }
    else if (p > 0)
    {
        is_dg_proc_dead = true;
    }
    else
    {
        if (DYNAMIC_GRAPH_MANAGER_VERBOSE)
        {
            printf(
                "DynamicGraphManager::has_dynamic_graph_process_died():"
                " waitpid failed\n");
        }
        is_dg_proc_dead = true;
    }
    return is_dg_proc_dead;
}

void DynamicGraphManager::initialize_dynamic_graph_process()
{
    // export the robot name and the log directory from the yaml to the shared
    // memory so we can access it in the python interpretor.
    std::string robot_name = params_["device"]["name"].as<std::string>();
    shared_memory::set<std::string>(
        shared_memory_name_, "device_name", robot_name);
    shared_memory::set<std::string>(shared_memory_name_, "log_dir", log_dir_);

    // we create the device of the DG and implicitly the DG itself
    device_ = std::make_unique<Device>(robot_name);
    device_->initialize(params_["device"]);

    // we start the ros services for the DGM (python command + start/stop DG)
    start_ros_service();

    // we call the prologue of the python interpreter
    // we *NEED* to do this *AFTER* the device is created to fetch its pointer
    // in the python interpreter
    python_prologue();

    // we build the condition variables after the fork (seems safer this way)
    cond_var_ = std::make_unique<shared_memory::LockedConditionVariable>(
        cond_var_name_, false);
}

void DynamicGraphManager::run_python_command(std::ostream& file,
                                             const std::string& command)
{
    file << ">>> " << command << std::endl;
    std::string lerr(""), lout(""), lres("");
    ros_python_interpreter_->run_python_command(command, lres, lout, lerr);
    if (lres != "None")
    {
        if (lres == "<NULL>")
        {
            file << lout << std::endl;
            file << "------" << std::endl;
            file << lerr << std::endl;
        }
        else
        {
            file << lres << std::endl;
        }
    }
}

void DynamicGraphManager::python_prologue()
{
    // open the log file
    std::ofstream aof(python_log_file_.c_str());

    run_python_command(aof,
                       "print(\"Executing python interpreter prologue...\")");
    // make sure that the current environment variable are setup in the current
    // python interpreter.
    run_python_command(aof, "import sys, os");
    run_python_command(aof, "print(\"python version:\", sys.version)");
    run_python_command(aof, "pythonpath = os.environ['PYTHONPATH']");
    run_python_command(aof, "path = []");
    run_python_command(aof,
                       "for p in pythonpath.split(':'):\n"
                       "  if p not in sys.path:\n"
                       "    path.append(p)");
    run_python_command(aof, "path.extend(sys.path)");
    run_python_command(aof, "sys.path = path");
    // used to be able to invoke rospy
    run_python_command(aof,
                       "if not hasattr(sys, \'argv\'):\n"
                       "    sys.argv  = ['dynamic_graph_manager']");
    // Create the device or get a pointer to the c++ object if it already exist
    run_python_command(aof, "from dynamic_graph_manager.prologue import robot");

    run_python_command(
        aof, "print(\"Executing python interpreter prologue... Done\")");
    // close the log file
    aof.close();
}

void DynamicGraphManager::run_dynamic_graph_process()
{
    printf("wait to start dynamic graph\n");
    if (ros_ok())
    {
        // launch the real time thread
        thread_dynamic_graph_.reset(new real_time_tools::RealTimeThread());
        thread_dynamic_graph_->parameters_.cpu_id_.push_back(0);  // cpu 0
        thread_dynamic_graph_->create_realtime_thread(
            &DynamicGraphManager::dynamic_graph_real_time_loop_helper, this);
        printf("dynamic graph thread started\n");
    }
    else
    {
        printf("dynamic graph thread NOT started as ROS has been shutdown.\n");
    }
}

void DynamicGraphManager::run_hardware_communication_process()
{
    // from here on this process is a ros node
    get_ros_node(hw_com_ros_node_name_);
    ros_add_node_to_executor(hw_com_ros_node_name_);

    // we build the condition variables after the fork (seems safer this way)
    cond_var_.reset(
        new shared_memory::LockedConditionVariable(cond_var_name_, true));

    // allow the hardware thread to run
    start_hardware_communication();

    // launch the real time thread and ros spin
    std::vector<int> cpu_affinity;
    cpu_affinity.clear();
    cpu_affinity.push_back(2);  // cpu 1
    thread_hardware_communication_.reset(new real_time_tools::RealTimeThread());
    thread_hardware_communication_->parameters_.cpu_id_.push_back(2);  // cpu 2
    thread_hardware_communication_->create_realtime_thread(
        &DynamicGraphManager::hardware_communication_real_time_loop_helper,
        this);
    printf("hardware communication loop started\n");
}

void DynamicGraphManager::run_single_process()
{
    printf("wait to start dynamic graph\n");
    wait_start_dynamic_graph();

    // launch the real time thread and ros spin
    thread_dynamic_graph_.reset(new real_time_tools::RealTimeThread());
    thread_dynamic_graph_->create_realtime_thread(
        &DynamicGraphManager::single_process_real_time_loop_helper, this);

    printf("single process dynamic graph loop started\n");
}

void DynamicGraphManager::start_ros_service()
{
    // Advertize the service to start and stop the dynamic graph
    RosNodePtr ros_node_handle = get_ros_node(dg_ros_node_name_);
    ros_add_node_to_executor(dg_ros_node_name_);
    ros_service_start_dg_ =
        ros_node_handle->create_service<std_srvs::srv::Empty>(
            "start_dynamic_graph",
            std::bind(static_cast<void (DynamicGraphManager::*)(
                          std_srvs::srv::Empty::Request::SharedPtr,
                          std_srvs::srv::Empty::Response::SharedPtr)>(
                          &DynamicGraphManager::start_dynamic_graph),
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));
    ros_service_stop_dg_ =
        ros_node_handle->create_service<std_srvs::srv::Empty>(
            "stop_dynamic_graph",
            std::bind(static_cast<void (DynamicGraphManager::*)(
                          std_srvs::srv::Empty::Request::SharedPtr,
                          std_srvs::srv::Empty::Response::SharedPtr)>(
                          &DynamicGraphManager::stop_dynamic_graph),
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));
    // advertize the ros::services associated to the python interpreter
    // we create a python interpreter
    ros_python_interpreter_ = std::make_unique<RosPythonInterpreterServer>();
    ros_python_interpreter_->start_ros_service();
}

void* DynamicGraphManager::dynamic_graph_real_time_loop()
{
    // std::cout << "DG: Locking scope..." << std::endl;
    cond_var_->lock_scope();
    get_ros_node(dg_ros_node_name_);

    wait_start_dynamic_graph();
    rt_printf("DG: Start loop\n");

    // initialize the timers
    dg_active_timer_.tic();
    dg_sleep_timer_.tic();
    dg_timer_.tic();

    while (!is_dynamic_graph_stopped() && ros_ok())
    {
        // measure the complete iteration time
        dg_timer_.tac_tic();
        // measure the active time
        dg_active_timer_.tic();
        // read the sensor from the shared memory
        shared_memory::get(
            shared_memory_name_, sensors_map_name_, sensors_map_);

        // call the dynamic graph
        device_->set_sensors_from_map(sensors_map_);
        device_->execute_graph();
        device_->get_controls_to_map(motor_controls_map_);

        // write the command to the shared memory
        shared_memory::set(
            shared_memory_name_, motor_controls_map_name_, motor_controls_map_);

        // measure the active time
        dg_active_timer_.tac();
        // measure the sleep time
        dg_sleep_timer_.tic();

        // notify the hardware_communication process that the control has been
        // done
        cond_var_->notify_all();

        // wait that the hardware_communication process acquiers the data.
        cond_var_->wait();

        // measure the sleep time
        dg_sleep_timer_.tac();
    }

    // we use this function here because the loop might stop because of ROS
    stop_dynamic_graph();

    rt_printf("DG: Dumping time measurement\n");
    dg_active_timer_.dump_measurements(dg_active_timer_file_);
    dg_sleep_timer_.dump_measurements(dg_sleep_timer_file_);
    dg_timer_.dump_measurements(dg_timer_file_);

    cond_var_->unlock_scope();
    rt_printf("DG: Stop Loop\n");

    return THREAD_FUNCTION_RETURN_VALUE;
}

void* DynamicGraphManager::hardware_communication_real_time_loop()
{
    // we acquiere the lock on the condition variable here
    cond_var_->lock_scope();

    // some basic checks
    assert(!is_hardware_communication_stopped_ && "The loop is started");
    get_ros_node(hw_com_ros_node_name_);
    assert(ros_ok() && "Ros has to be initialized");

    // Initialize the motor_controls_map with zeros.
    for (VectorDGMap::iterator ctrl = motor_controls_map_.begin();
         ctrl != motor_controls_map_.end();
         ++ctrl)
    {
        ctrl->second.fill(0.0);
    }

    // Initialize the time measurements
    hwc_active_timer_.tic();
    hwc_timer_.tic();

    // time the loop so it respect the input frequence/period
    hwc_spinner_.set_period(control_period_sec_);
    hwc_spinner_.initialize();

    // we start the main loop
    rt_printf("HARDWARE: Start loop \n");
    hwc_mutex_.lock();
    while (!is_hardware_communication_stopped() && ros_ok())
    {
        // call the sensors
        if (!is_hardware_communication_stopped() && ros_ok())
        {
            get_sensors_to_map(sensors_map_);
        }

        if (cond_var_->owns() || cond_var_->try_lock())
        {
            // write the sensors to the shared memory
            shared_memory::set(
                shared_memory_name_, sensors_map_name_, sensors_map_);

            // If running on the real hardware, unlock the mutex such that the
            // task process can acquire it right away after calling
            // `notify_all()`. In simulation, the mutex is released by the call
            // to `wait` further below.
            cond_var_->unlock();

            // Notify the dynamic graph process that it can compute the graph.
            // If running on the real hardware, this notify_all also acts as
            // time-synchronization between the task and motor process.
            cond_var_->notify_all();
        }

        // predict here the sleeping time in the spinner
        hwc_predicted_sleeping_time_ = hwc_spinner_.predict_sleeping_time();
        if (hwc_predicted_sleeping_time_ > maximum_time_for_user_cmd_)
        {
            if (user_commands_.size() > 0)
            {
                user_commands_[0]();
                user_commands_.pop_front();
                rt_printf("HARDWARE: Executed user command\n");
            }
        }

        // here is the end of the thread activity
        hwc_active_timer_.tac();
        hwc_timer_.tac_tic();

        // Sleeps for one period. This clocks the motor process.
        hwc_sleep_timer_.tic();
        hwc_mutex_.unlock();
        hwc_spinner_.spin();
        hwc_mutex_.lock();
        hwc_sleep_timer_.tac();

        // here is the beginning of the thread activity
        hwc_active_timer_.tic();

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
            static bool once = false;
            if (!once)
            {
                rt_printf("HARDWARE: Warning enter into safe_mode\n");
                once = true;
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
    stop_hardware_communication();

    rt_printf("HARDWARE: Dump active time measurement \n");
    hwc_active_timer_.dump_measurements(hwc_active_timer_file_);
    rt_printf("HARDWARE: sleep time measurement \n");
    hwc_sleep_timer_.dump_measurements(hwc_sleep_timer_file_);
    rt_printf("HARDWARE: hwc time measurement \n");
    hwc_timer_.dump_measurements(hwc_timer_file_);
    cond_var_->unlock_scope();
    rt_printf("HARDWARE: Stop loop \n");

    return THREAD_FUNCTION_RETURN_VALUE;
}

void DynamicGraphManager::compute_safety_controls()
{
    // VERY STUPID!!! TODO: FIND A BETTER WAY AND USE AT LEAST PID CONTROL
    for (VectorDGMap::iterator ctrl = motor_controls_map_.begin();
         ctrl != motor_controls_map_.end();
         ++ctrl)
    {
        ctrl->second.fill(0.0);
    }
}

void* DynamicGraphManager::single_process_real_time_loop()
{
    // //std::cout << "DG: Locking scope..." << std::endl;
    std::cout << "DG: Start loop" << std::endl;
    while (!is_dynamic_graph_stopped() && ros_ok())
    {
        // acquire the sensors data
        get_sensors_to_map(sensors_map_);

        // call the dynamic graph
        device_->set_sensors_from_map(sensors_map_);
        device_->execute_graph();
        device_->get_controls_to_map(motor_controls_map_);

        // send the command to the motors
        set_motor_controls_from_map(motor_controls_map_);
    }
    // we use this function here because the loop might stop because of ROS
    // we need to make sure that the flag is set properly
    stop_dynamic_graph();
    // printf("dynamic graph thread stopped\n");
    std::cout << "DG: Stop loop" << std::endl;

    return THREAD_FUNCTION_RETURN_VALUE;
}

void DynamicGraphManager::add_user_command(std::function<void(void)> func)
{
    hwc_mutex_.lock();
    user_commands_.push_back(func);
    hwc_mutex_.unlock();
}
