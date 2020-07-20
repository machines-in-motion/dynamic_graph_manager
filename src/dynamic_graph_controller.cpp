/**
 * @file dynamic_graph_manager.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

// use for real_time printf
#include "real_time_tools/iostream.hpp"
// use to set cpu latency.
#include "real_time_tools/process_manager.hpp"
// use the ROS singleton to initialize and use ROS.
#include "dynamic_graph_manager/ros_init.hpp"
// this file defines the class in this header
#include "dynamic_graph_manager/dynamic_graph_controller.hpp"

#define DYNAMIC_GRAPH_MANAGER_VERBOSE 0

using namespace dynamic_graph_manager;

const std::string DynamicGraphController::ros_node_name_ = "dynamic_graph";

DynamicGraphController::DynamicGraphController()
    : ros_node_(ros_init(ros_node_name_))
{
    // Upon construction the graph is inactive
    params_.reset();

    stop();
    destructor_called_ = false;

    sensors_map_.clear();
    joint_controls_map_.clear();

    // clean the shared memory
    shared_memory::clear_shared_memory(dgm_shared_memory::shared_memory_name);

    // files where to dump the timers
    log_dir_ = "/tmp/";
    active_timer_file_ = "/tmp/DynamicGraphController_active_timer.dat";
    sleep_timer_file_ = "/tmp/DynamicGraphController_sleep_timer.dat";
    timer_file_ = "/tmp/DynamicGraphController_timer.dat";
    python_log_file_ = "/tmp/DynamicGraphController_python_instruction.txt";
}

DynamicGraphController::~DynamicGraphController()
{
    destructor_called_ = true;
    // wait for the dynamic graph thread to stop
    stop();
    if (cond_var_)
    {
        cond_var_->notify_all();
    }
    wait_until_controller_stops();
    std::cout << "All done!" << std::endl;
}

void DynamicGraphController::initialize(YAML::Node param)
{
    // Upon initialization the graph is inactive
    stop();

    // Copy the yaml node for further use
    params_ = param;

    // We initialize the sensors and control maps in the common initializer
    parse_yaml_node(params_["device"], sensors_map_, joint_controls_map_);
    shared_memory::set(dgm_shared_memory::shared_memory_name,
                       dgm_shared_memory::sensors_map_name,
                       sensors_map_);
    shared_memory::set(dgm_shared_memory::shared_memory_name,
                       dgm_shared_memory::joint_controls_map_name,
                       joint_controls_map_);

    // Set the maximum cpu latency to 0 us. This keeps the CPU from sleeping.
    real_time_tools::set_cpu_dma_latency(0);

    // Get the log folder
    log_dir_ = real_time_tools::get_log_dir("dynamic_graph_manager");
    active_timer_file_ = log_dir_ + "DynamicGraphController_active_timer.dat";
    sleep_timer_file_ = log_dir_ + "DynamicGraphController_sleep_timer.dat";
    timer_file_ = log_dir_ + "DynamicGraphController_timer.dat";
    python_log_file_ =
        log_dir_ + "DynamicGraphController_python_instruction.txt";

    unsigned int debug_timer_history_length = 0;
    YAML::ReadParameter(params_,
                        "debug_timer_history_length",
                        debug_timer_history_length,
                        true);

    active_timer_.set_memory_size(debug_timer_history_length);
    sleep_timer_.set_memory_size(debug_timer_history_length);
    timer_.set_memory_size(debug_timer_history_length);

    std::cout << "DynamicGraphController logs will be saved in : \"" << log_dir_
              << "\"" << std::endl;

    // Get the name of the device.
    std::string robot_name = params_["device"]["name"].as<std::string>();

    // Export the yaml node to ros so we can access it in the python interpretor
    // and in other ros node if needed.
    ros_node_.setParam("device_name", robot_name);
    ros_node_.setParam("log_dir", log_dir_);

    // We create the device entity and implicitly the Dynamic Graph itself.
    device_ = std::make_unique<Device>(robot_name);
    device_->initialize(params_["device"]);

    // we create a python interpreter
    ros_python_interpreter_ = std::make_unique<RosPythonInterpreter>(ros_node_);
    assert(ros_python_interpreter_);
    // we call the prologue of the python interpreter
    // we *NEED* to do this *AFTER* the device is created to fetch its pointer
    // in the python interpreter
    python_prologue();

    // we start the ros services for the DGM (python command + start/stop DG)
    advertise_rosservice();

    // we build the condition variables after the fork (seems safer this way)
    cond_var_ = std::make_unique<shared_memory::LockedConditionVariable>(
        dgm_shared_memory::cond_var_name, false);
}

void DynamicGraphController::run()
{
    rt_printf("DynamicGraphController: Spawning thread.\n");
    if (ros_node_.ok())
    {
        // launch the real time thread
        rt_thread_ = std::make_unique<real_time_tools::RealTimeThread>();
        rt_thread_->parameters_.cpu_id_.push_back(0);  // cpu 0
        rt_thread_->create_realtime_thread(
            &DynamicGraphController::controller_real_time_loop_helper, this);
        rt_printf("DynamicGraphController: Thread spawned.\n");
    }
    else
    {
        rt_printf(
            "DynamicGraphController: Thread NOT spawned as ROS has been "
            "shutdown.\n");
    }
}

void DynamicGraphController::wait_until_controller_starts()
{
    ros::NodeHandle& ros_node = ros_init(ros_node_name_);
    while (is_stopped() && ros_node.ok() && !destructor_called_)
    {
        usleep(1000);
    }
}

void DynamicGraphController::wait_until_controller_stops()
{
    while (!is_stopped() && ros_node_.ok())
    {
        usleep(100000);
    }
    stop();
    if (rt_thread_)
    {
        cond_var_->notify_all();
        rt_thread_->join();
    }
}

void DynamicGraphController::run_python_command(std::ostream& file,
                                                const std::string& command)
{
    file << ">>> " << command << std::endl;
    std::string lerr(""), lout(""), lres("");
    assert(ros_python_interpreter_);
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

void DynamicGraphController::python_prologue()
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
    // used to be able to invoke rospkg
    run_python_command(aof,
                       "if not hasattr(sys, \'argv\'):\n"
                       "    sys.argv  = ['dynamic_graph_manager']");
    // Create the device or get a pointer to the c++ object.
    run_python_command(
        aof, "from dynamic_graph_manager.device.prologue import robot");

    run_python_command(
        aof, "print(\"Executing python interpreter prologue... Done\")");
    aof.close();
}

void DynamicGraphController::advertise_rosservice()
{
    // Advertize the service to start and stop the dynamic graph
    rosservice_start_ = ros_node_.advertiseService(
        "start", &DynamicGraphController::start, this);
    rosservice_stop_ =
        ros_node_.advertiseService("stop", &DynamicGraphController::stop, this);
    // advertize the ros::services associated to the python interpreter
    ros_python_interpreter_->start_ros_service();
}

void DynamicGraphController::controller_real_time_loop()
{
    cond_var_->lock_scope();
    rt_printf("DynamicGraphController: Wait to start.\n");
    wait_until_controller_starts();
    rt_printf("DynamicGraphController: Start loop\n");

    // initialize the timers
    active_timer_.tic();
    sleep_timer_.tic();
    timer_.tic();

    while (!is_stopped_ && ros_node_.ok())
    {
        // Measure the complete iteration time.
        timer_.tac_tic();
        // Measure the active time.
        active_timer_.tic();
        // Read the sensor from the shared memory.
        shared_memory::get(dgm_shared_memory::shared_memory_name,
                           dgm_shared_memory::sensors_map_name,
                           sensors_map_);

        // Call the dynamic graph.
        device_->set_sensors_from_map(sensors_map_);
        device_->execute_graph();
        device_->get_controls_to_map(joint_controls_map_);

        // Write the command to the shared memory.
        shared_memory::set(dgm_shared_memory::shared_memory_name,
                           dgm_shared_memory::joint_controls_map_name,
                           joint_controls_map_);

        // Measure the active time.
        active_timer_.tac();
        // Measure the sleep time.
        sleep_timer_.tic();

        // Notify the hardware_communication process that the control has been
        // done.
        cond_var_->notify_all();

        // Wait that the hardware_communication process acquiers the data.
        cond_var_->wait();

        // Measure the sleep time.
        sleep_timer_.tac();
    }

    // We use this function here because the loop might stop because of ROS.
    stop();

    // Zero the control and set it to the shared_memeory.
    for (VectorDGMap::iterator control_it = joint_controls_map_.begin();
         control_it != joint_controls_map_.end();
         ++control_it)
    {
        control_it->second.setZero();
    }
    shared_memory::set(dgm_shared_memory::shared_memory_name,
                       dgm_shared_memory::joint_controls_map_name,
                       joint_controls_map_);
    // Notify the hardware_communication process that the control has been
    // done.
    cond_var_->notify_all();

    rt_printf("DynamicGraphController: Dumping time measurement\n");
    active_timer_.dump_measurements(active_timer_file_);
    sleep_timer_.dump_measurements(sleep_timer_file_);
    timer_.dump_measurements(timer_file_);

    cond_var_->unlock_scope();
    rt_printf("DynamicGraphController: Stop Loop\n");

    return;
}
