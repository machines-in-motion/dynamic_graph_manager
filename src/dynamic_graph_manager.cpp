/**
 * \file dynamic_graph_manager.cpp
 * \brief The Control Manager
 * \author Maximilien Naveau
 * \date 2018
 *
 * Implementation of the DynamicGraphManager class
 *
 */

#include "real_time_tools/spinner.h"
#include "real_time_tools/realtime_check.h"
#include <dynamic_graph_manager/ros_init.hh>
#include <dynamic_graph_manager/dynamic_graph_manager.hh>

using namespace dynamic_graph;

const std::string DynamicGraphManager::dg_ros_node_name_ = "dynamic_graph";
const std::string DynamicGraphManager::hw_com_ros_node_name_ =
    "hardware_communication";
const std::string DynamicGraphManager::python_log_file_ =
     "/tmp/python_log_dynamic_graph_manager.out";

DynamicGraphManager::DynamicGraphManager()
{
  // Upon construction the graph is inactive
  params_.reset();

  stop_dynamic_graph();
  stop_hardware_communication();

  is_real_robot_ = true;

  pid_dynamic_graph_process_ = 0;
  pid_hardware_communication_process_ = 0;

  shared_memory_name_ = "DGM_ShM";
  sensors_map_name_ = "sensors_map";
  motor_controls_map_name_ = "motor_controls_map";
  cond_var_name_ = "cond_var";

  sensors_map_.clear();
  motor_controls_map_.clear();

  has_been_waken_by_dg_ = false;

  missed_control_count_ = 0;
  max_missed_control_ = 0;
  control_period_ = clock::duration(0);
}

DynamicGraphManager::~DynamicGraphManager()
{
  // kill all ros related stuff
  ros_service_start_dg_.shutdown();
  ros_service_stop_dg_.shutdown();
  ros_shutdown();
  // wait for the dynamic graph thread to stop
  stop_dynamic_graph();
  if(cond_var_)
  {
    cond_var_->notify_all();
  }
  wait_stop_dynamic_graph();
  // wait for the hardware communication thread to stop
  stop_hardware_communication();
  if(cond_var_)
  {
    cond_var_->notify_all();
  }
  wait_stop_hardware_communication();
  // clean the shared memory
  shared_memory::clear_shared_memory(shared_memory_name_);
  // destroy the python interpretor before the device

  device_.reset(nullptr);
  ros_python_interpreter_.reset(nullptr);
}

void DynamicGraphManager::initialize(YAML::Node param){

  // Upon initialization the graph and the hardware com are inactive
  stop_dynamic_graph();
  stop_hardware_communication();

  // copy the yaml node for further use
  params_ = param ;

  pid_dynamic_graph_process_ = 0;
  pid_hardware_communication_process_ = 0;

  // we initialize the sensors and control maps in the common initializer
  parse_yaml_node(params_["device"], sensors_map_, motor_controls_map_);
  shared_memory::set(shared_memory_name_, sensors_map_name_, sensors_map_);
  shared_memory::set(shared_memory_name_, motor_controls_map_name_,
                     motor_controls_map_);

  // get the paarameter for the hardware communication loop
  max_missed_control_ = params_["hardware_communication"]
                        ["max_missed_control"].as<unsigned>();
  control_period_ =  clock::duration(params_["hardware_communication"]
                     ["control_period"].as<unsigned>());

  is_real_robot_ = params_["is_real_robot"].as<bool>();

  // we create and destroy the condition variable to free the shared memory
  // and therefore the associated mutex which must be lockable at this state.
  {
    shared_memory::ConditionVariable(shared_memory_name_,
                                     cond_var_name_);
  }
}

void DynamicGraphManager::run()
{
  pid_t child_pid = fork();
  if(child_pid == 0) // child process
  {
    pid_dynamic_graph_process_ = getpid();
    pid_hardware_communication_process_ = getppid();

    initialize_dynamic_graph_process();
    run_dynamic_graph_process();
    wait_stop_dynamic_graph();
    ros::waitForShutdown();
    std::cout << "End of the dynamic graph process." << std::endl;
    exit(0);
  }else if(child_pid > 0) // parent process
  {
    pid_dynamic_graph_process_ = child_pid;
    pid_hardware_communication_process_ = getpid();

    initialize_hardware_communication_process();
    run_hardware_communication_process();
  }else
  {
    throw(std::runtime_error("DynamicGraphManager::run(): the fork failed"));
  }
}

void DynamicGraphManager::wait_start_dynamic_graph()
{
  while(is_dynamic_graph_stopped())
  {
    usleep(1000);
  }
}

void DynamicGraphManager::wait_stop_dynamic_graph()
{
  while(!is_dynamic_graph_stopped())
  {
    usleep(100000);
  }
  if(thread_dynamic_graph_)
  {
    cond_var_->notify_all();
    real_time_tools::join_thread(*thread_dynamic_graph_);
  }
}

void DynamicGraphManager::wait_stop_hardware_communication()
{
  while(!is_hardware_communication_stopped())
  {
    usleep(100000);
  }
  if(thread_hardware_communication_)
  {
    cond_var_->notify_all();
    real_time_tools::join_thread(*thread_hardware_communication_);
  }
}

bool DynamicGraphManager::has_dynamic_graph_process_died()
{
  bool is_dg_proc_dead = false;
  int status = 0;
  pid_t p = 0;
  p = waitpid(pid_dynamic_graph_process_, &status, WNOHANG);
  if(p == 0)
  {
    is_dg_proc_dead = false;
  }
  else if(p > 0)
  {
    is_dg_proc_dead = true;
  }
  else
  {
    printf("DynamicGraphManager::has_dynamic_graph_process_died():"
           " waitpid failed\n");
    is_dg_proc_dead = true;
  }
  return is_dg_proc_dead;
}

void DynamicGraphManager::initialize_dynamic_graph_process()
{
  // from here this process becomes a ros node
  ros::NodeHandle& ros_node_handle = ros_init(dg_ros_node_name_);
  // we create a python interpreter
  ros_python_interpreter_.reset(
        new dynamic_graph::RosPythonInterpreter(ros_node_handle));
  // we start the ros services for the DGM (python command + start/stop DG)
  start_ros_service(ros_node_handle);
  // we create the device of the DG and implicitly the DG itself
  device_.reset(new Device(params_["device"]["name"].as<std::string>(),
      params_["device"]));
  // we build the condition variables after the fork (seems safer this way)
  cond_var_.reset(new shared_memory::ConditionVariable(
                            shared_memory_name_,
                            cond_var_name_));
  // we call the prologue of the python interpreter
  // we *NEED* to do this *AFTER* the device is created to fetch its pointer
  // in the python interpreter
  python_prologue();
}

void DynamicGraphManager::run_python_command(std::ostream& file,
                                             const std::string& command)
{
  file << ">>> " << command << std::endl;
  std::string lerr(""),lout(""),lres("");
  ros_python_interpreter_->run_python_command(command,lres,lout,lerr);
  if (lres != "None")
  {
    if (lres=="<NULL>")
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

  run_python_command(
        aof, "print(\"Executing python interpreter prologue...\")");
  // make sure that the current environment variable are setup in the current
  // python interpreter.
  run_python_command(aof, "import sys, os");
  run_python_command(aof, "pythonpath = os.environ['PYTHONPATH']");
  run_python_command(aof, "path = []");
  run_python_command(aof,
             "for p in pythonpath.split(':'):\n"
             "  if p not in sys.path:\n"
             "    path.append(p)");
  run_python_command (aof, "path.extend(sys.path)");
  run_python_command (aof, "sys.path = path");
  // used to be able to invoke rospy
  run_python_command (aof,
                      "if not hasattr(sys, \'argv\'):\n"
                      "    sys.argv  = ['dynamic_graph_manager']");
  // Create the device or get a pointer to the c++ object if it already exist
  run_python_command(aof, "from dynamic_graph.device.prologue import robot");
  run_python_command(
        aof, "print(\"Executing python interpreter prologue... Done\")");
  // close the log file
  aof.close();
}

void DynamicGraphManager::run_dynamic_graph_process()
{
  printf("wait to start dynamic graph\n");
  wait_start_dynamic_graph();
  // launch the real time thread and ros spin
  real_time_tools::block_memory();
  thread_dynamic_graph_.reset(new real_time_tools::RealTimeThread());
  real_time_tools::create_realtime_thread(
        *thread_dynamic_graph_,
        &DynamicGraphManager::dynamic_graph_real_time_loop_helper, this);
  printf("dynamic graph thread started\n");
}

void DynamicGraphManager::run_hardware_communication_process()
{
  // from here on this process is a ros node
  ros::NodeHandle& hw_ros_node = ros_init(hw_com_ros_node_name_);

  // export the yaml node to ros so we can access it in the python interpretor
  // and in other ros node if needed.
  hw_ros_node.setParam("device_name",
                       params_["device"]["name"].as<std::string>());

  // we build the condition variables after the fork (seems safer this way)
  cond_var_.reset(new shared_memory::ConditionVariable(
                            shared_memory_name_,
                            cond_var_name_));

  // allow the hardware thread to run
  start_hardware_communication();

  // launch the real time thread
  real_time_tools::block_memory();
  thread_hardware_communication_.reset(new real_time_tools::RealTimeThread());
  real_time_tools::create_realtime_thread(
        *thread_hardware_communication_,
        &DynamicGraphManager::hardware_communication_real_time_loop_helper,
        this);
  printf("hardware communication loop started\n");
}

void DynamicGraphManager::start_ros_service(ros::NodeHandle& ros_node_handle)
{
  // Advertize the service to start and stop the dynamic graph
  ros_service_start_dg_ = ros_node_handle.advertiseService(
                              "start_dynamic_graph",
                              &DynamicGraphManager::start_dynamic_graph,
                              this);
  ros_service_stop_dg_ = ros_node_handle.advertiseService(
                             "stop_dynamic_graph",
                             &DynamicGraphManager::stop_dynamic_graph,
                             this);
  // advertize the ros::services associated to the python interpreter
  ros_python_interpreter_->start_ros_service();
}

void* DynamicGraphManager::dynamic_graph_real_time_loop()
{
  cond_var_->lock_scope();

  // //std::cout << "DG: Locking scope..." << std::endl;
  std::cout << "DG: Start loop" << std::endl;
  while(!is_dynamic_graph_stopped() && ros::ok())
  {
    // read the sensor from the shared memory
    shared_memory::get(shared_memory_name_, sensors_map_name_, sensors_map_);

    // call the dynamic graph
    device_->set_sensors_from_map(sensors_map_);
    device_->execute_graph();
    device_->get_controls_to_map(motor_controls_map_);

    // write the command to the shared memory
    shared_memory::set(shared_memory_name_, motor_controls_map_name_,
                       motor_controls_map_);

    // notify the hardware_communication process that the control has been done
    cond_var_->notify_all();

    // wait that the hardware_communication process acquiers the data.
    cond_var_->wait();
  }
  // we use this function here because the loop might stop because of ROS
  stop_dynamic_graph();
  //printf("dynamic graph thread stopped\n");
  std::cout << "DG: Stop loop" << std::endl;
  cond_var_->unlock_scope();
}

void* DynamicGraphManager::hardware_communication_real_time_loop()
{
  // we acquiere the lock on the condition variable here
  cond_var_->lock_scope();

  // some basic checks
  assert(!is_hardware_communication_stopped_ && "The loop is started");
  assert(ros::ok() && "Ros has to be initialized");

  std::cout << "HARDWARE: Start loop" << std::endl;

  // HACK: Call this method to initialize the motor_controls_map with zeros.
  compute_safety_controls();

  double control_frequency_ = 1e9 / static_cast<double>(control_period_.count());
  real_time_tools::Spinner spinner(control_frequency_);
  // real_time_tools::Realtime_check realtime_check(control_frequency_);

  // we start the main loop
  while(!is_hardware_communication_stopped() && ros::ok())
  {
    //std::cout << "HARDWARE: loop" << std::endl;
    // call the sensors
    get_sensors_to_map(sensors_map_);

    if (cond_var_->owns() || cond_var_->try_lock()) {
      // write the sensors to the shared memory
      shared_memory::set(shared_memory_name_, sensors_map_name_, sensors_map_);

      // If running on the real hardware, unlock the mutex such that the
      // task process can acquire it right away after calling `notify_all()`.
      // In simulation, the mutex is released by the call to `wait` further below.
      if (is_real_robot_) {
        cond_var_->unlock();
      }

      // Notify the dynamic graph process that it can compute the graph.
      // If running on the real hardware, this notify_all also acts as
      // time-synchronization between the task and motor process.
      cond_var_->notify_all();
    }

    // NOTE(jviereck): SL reads the current command values at this point and
    //                 sends them to the robot before going to sleep.

    if (is_real_robot_) {
      // Sleeps for one period. This clocks the motor process.
      spinner.spin();

      // TODO: Check if realtime was lost here. Using realtime_check always
      // recorded switches for jviereck.

      // If the control process finished its computation, it is waiting
      // and we are able to acquire the lock here.
      if (cond_var_->try_lock()) {
        // The a new control command is available.
        missed_control_count_ = 0;
      } else {
        ++missed_control_count_;
      }

      if (is_in_safety_mode()) {
        compute_safety_controls();
        // we write the safety command in the shared memory in order to perform
        // some interpolation
        // QUESTION(jviereck): Why write the values into the shared memory here?
        //                     Couldn't this cause the values written by the
        //                     task proces to get overwritten?
        shared_memory::set(shared_memory_name_, motor_controls_map_name_,
                           motor_controls_map_);
      } else if (missed_control_count_ == 0) {
        // If the wait on the conditional variable was successful, then read
        // the values from shared memory.
        shared_memory::get(shared_memory_name_, motor_controls_map_name_,
                           motor_controls_map_);
     }
    } else {
        // if we are in simulation we stop here until we actually get a control.
        cond_var_->wait();
        shared_memory::get(shared_memory_name_, motor_controls_map_name_,
                           motor_controls_map_);
    }


    // we do not send the command if the thread is asked to stopped
    if(!is_hardware_communication_stopped() && ros::ok()) {
      // send the command to the motors
      set_motor_controls_from_map(motor_controls_map_);
    }

  }
  // we use this function here because the loop might stop because of ROS
  stop_hardware_communication();
  //printf("hardware communication loop stopped\n");
  std::cout << "HARDWARE: Stop loop" << std::endl;
  cond_var_->unlock_scope();
}

void DynamicGraphManager::compute_safety_controls()
{
  // VERY STUPID!!! TODO: FIND A BETTER WAY AND USE AT LEAST PID CONTROL
  for(VectorDGMap::iterator ctrl = motor_controls_map_.begin() ;
      ctrl != motor_controls_map_.end() ; ++ctrl)
  {
    ctrl->second.fill(0.0);
  }
}
