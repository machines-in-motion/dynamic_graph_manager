/**
 * \file dynamic_graph_manager.cpp
 * \brief The Control Manager
 * \author Maximilien Naveau
 * \date 2018
 *
 * Implementation of the DynamicGraphManager class
 *
 */

// use the realtime spinner to time the loops
#include <real_time_tools/spinner.hpp>
// use the realtime checks to measure the loops computation time
#include <real_time_tools/realtime_check.hpp>
// use for real_time printf
#include <real_time_tools/realtime_iostream.hpp>
// use the ROS singleton to initialize and use ROS
#include <dynamic_graph_manager/ros_init.hh>
// this file defines the class in this header
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

  // clean the shared memory
  shared_memory::clear_shared_memory(shared_memory_name_);

  // files where to dump the timers
  log_dir_ = "/tmp/";
  dg_timer_file_ = "/tmp/dg_timer.dat";
  hwc_active_timer_file_ = "/tmp/hwc_active_timer.dat";
  hwc_sleep_timer_file_ = "/tmp/hwc_sleep_timer.dat";
  hwc_timer_file_ = "/tmp/hwc_timer.dat";
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
  // destroy the python interpretor after the device
  device_.reset(nullptr);
  ros_python_interpreter_.reset(nullptr);
  if(pid_hardware_communication_process_ == getpid())
  {
    kill(pid_dynamic_graph_process(), SIGKILL);
    while(!has_dynamic_graph_process_died())
    {
      usleep(1000);
    }
  }
}

void DynamicGraphManager::initialize(YAML::Node param){

  // Upon initialization the graph and the hardware com are inactive
  stop_dynamic_graph();
  stop_hardware_communication();

  // copy the yaml node for further use
  params_ = param ;

  pid_dynamic_graph_process_ = 0;
  pid_hardware_communication_process_ = 0;

  // clean the shared memory
  shared_memory::clear_shared_memory(shared_memory_name_);
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
  control_period_sec_ = params_["hardware_communication"]
                           ["control_period"].as<double>() * std::pow(10,-9);

  is_real_robot_ = params_["is_real_robot"].as<bool>();

  try{
    log_dir_ = params_["log_dir"].as<std::string>();
    dg_timer_file_ = log_dir_ + params_["dg_timer_file"].as<std::string>();
    hwc_active_timer_file_ =
        log_dir_ + params_["hwc_active_timer_file"].as<std::string>();
    hwc_sleep_timer_file_ =
        log_dir_ + params_["hwc_sleep_timer_file"].as<std::string>();
    hwc_timer_file_ = log_dir_ + params_["hwc_timer_file"].as<std::string>();
    dg_timer_file_ = log_dir_ + params_["dg_timer_file"].as<std::string>();
  }catch(...){
    std::string home_dir = real_time_tools::get_home_dir();
    std::string app_dir = ".dynamic_graph_manager/";
    std::string date_dir = real_time_tools::Timer::get_current_date_str() + "/";
    log_dir_ = home_dir + app_dir + date_dir;
    real_time_tools::create_directory(home_dir + app_dir);
    real_time_tools::create_directory(log_dir_);
    dg_timer_file_ = log_dir_ + "dg_timer.dat";
    hwc_active_timer_file_ =  log_dir_ + "hwc_active_timer.dat";
    hwc_sleep_timer_file_ =  log_dir_ + "hwc_sleep_timer.dat";
    hwc_timer_file_ =  log_dir_ + "hwc_timer.dat";
  }
  std::cout << "Log will be saved in :" << log_dir_ << std::endl;

  // we create and destroy the condition variable to free the shared memory
  // and therefore the associated mutex which must be lockable at this state.
  {
    shared_memory::ConditionVariable(shared_memory_name_,
                                     cond_var_name_);
  }


}

void DynamicGraphManager::run()
{
  if(is_real_robot_)
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
  }else{
    initialize_dynamic_graph_process();
    initialize_hardware_communication_process();
    run_single_process();
  }
}

void DynamicGraphManager::wait_start_dynamic_graph()
{
  while(is_dynamic_graph_stopped() and ros::ok())
  {
    usleep(1000);
  }
}

void DynamicGraphManager::wait_stop_dynamic_graph()
{
  while(!is_dynamic_graph_stopped() && ros::ok())
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

  // export the yaml node to ros so we can access it in the python interpretor
  // and in other ros node if needed.
  ros_node_handle.setParam("device_name",
                           params_["device"]["name"].as<std::string>());
  ros_node_handle.setParam("log_dir", log_dir_);

  // we create a python interpreter
  ros_python_interpreter_.reset(
        new dynamic_graph::RosPythonInterpreter(ros_node_handle));
  // we start the ros services for the DGM (python command + start/stop DG)
  start_ros_service(ros_node_handle);
  // we create the device of the DG and implicitly the DG itself
  device_.reset(new Device(params_["device"]["name"].as<std::string>()));
  device_->initialize(params_["device"]);
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
  run_python_command(aof,
                     "from dynamic_graph_manager.device.prologue import robot");
  run_python_command(
        aof, "print(\"Executing python interpreter prologue... Done\")");
  // close the log file
  aof.close();
}

void DynamicGraphManager::run_dynamic_graph_process()
{
  printf("wait to start dynamic graph\n");
  wait_start_dynamic_graph();
  if(ros::ok())
  {
    // launch the real time thread and ros spin
    real_time_tools::block_memory();
    thread_dynamic_graph_.reset(new real_time_tools::RealTimeThread());
    real_time_tools::create_realtime_thread(
          *thread_dynamic_graph_,
          &DynamicGraphManager::dynamic_graph_real_time_loop_helper, this);
    printf("dynamic graph thread started\n");
  }else{
    printf("dynamic graph thread NOT started as ROS has been shutdown.\n");
  }
}

void DynamicGraphManager::run_hardware_communication_process()
{
  // from here on this process is a ros node
  ros::NodeHandle& hw_ros_node = ros_init(hw_com_ros_node_name_);

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

void DynamicGraphManager::run_single_process()
{
  printf("wait to start dynamic graph\n");
  wait_start_dynamic_graph();

  // launch the real time thread and ros spin
  real_time_tools::block_memory();
  thread_dynamic_graph_.reset(new real_time_tools::RealTimeThread());
  real_time_tools::create_realtime_thread(
        *thread_dynamic_graph_,
        &DynamicGraphManager::single_process_real_time_loop_helper, this);

  printf("single process dynamic graph loop started\n");
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
  //std::cout << "DG: Locking scope..." << std::endl;
  cond_var_->lock_scope();

  std::cout << "DG: Start loop" << std::endl;

  while(!is_dynamic_graph_stopped() && ros::ok())
  {
    dg_timer_.tic();
    // read the sensor from the shared memory
    shared_memory::get(shared_memory_name_, sensors_map_name_, sensors_map_);

    // call the dynamic graph
    device_->set_sensors_from_map(sensors_map_);
    device_->execute_graph();
    device_->get_controls_to_map(motor_controls_map_);

    // write the command to the shared memory
    shared_memory::set(shared_memory_name_, motor_controls_map_name_,
                       motor_controls_map_);

    dg_timer_.tac();

    // notify the hardware_communication process that the control has been done
    cond_var_->notify_all();

    // wait that the hardware_communication process acquiers the data.
    cond_var_->wait();
  }

  // we use this function here because the loop might stop because of ROS
  stop_dynamic_graph();

  std::cout << "DG: Dumping time measurement" << std::endl;
  dg_timer_.dump_measurements(dg_timer_file_);

  cond_var_->unlock_scope();
  std::cout << "DG: Stop Loop" << std::endl;
}

void* DynamicGraphManager::hardware_communication_real_time_loop()
{
  // we acquiere the lock on the condition variable here
  cond_var_->lock_scope();

  // some basic checks
  assert(!is_hardware_communication_stopped_ && "The loop is started");
  assert(ros::ok() && "Ros has to be initialized");

  rt_printf("HARDWARE: Start loop \n");

  // Initialize the motor_controls_map with zeros.
  for(VectorDGMap::iterator ctrl = motor_controls_map_.begin() ;
      ctrl != motor_controls_map_.end() ; ++ctrl)
  {
    ctrl->second.fill(0.0);
  }

  hwc_active_timer_.tic();
  hwc_timer_.tic();

  // time the loop so it respect the input frequence/period
  real_time_tools::Spinner spinner;
  spinner.set_period(control_period_sec_);

  // we start the main loop
  while(!is_hardware_communication_stopped() && ros::ok())
  {
    // call the sensors
    if(!is_hardware_communication_stopped() && ros::ok()) {
      get_sensors_to_map(sensors_map_);
    }

    if (cond_var_->owns() || cond_var_->try_lock()) {
      // write the sensors to the shared memory
      shared_memory::set(shared_memory_name_, sensors_map_name_, sensors_map_);

      // If running on the real hardware, unlock the mutex such that the
      // task process can acquire it right away after calling `notify_all()`.
      // In simulation, the mutex is released by the call to `wait` further below.
      cond_var_->unlock();

      // Notify the dynamic graph process that it can compute the graph.
      // If running on the real hardware, this notify_all also acts as
      // time-synchronization between the task and motor process.
      cond_var_->notify_all();
    }

    // here is the end of the thread activity
    hwc_active_timer_.tac();
    hwc_timer_.tac();
    hwc_timer_.tic();

    // Sleeps for one period. This clocks the motor process.
    hwc_sleep_timer_.tic();
    spinner.spin();
    hwc_sleep_timer_.tac();

    // here is the beginning of the thread activity
    hwc_active_timer_.tic();


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
      shared_memory::set(shared_memory_name_, motor_controls_map_name_,
                         motor_controls_map_);
    } else if (missed_control_count_ == 0) {
      // If the wait on the conditional variable was successful, then read
      // the values from shared memory.
      shared_memory::get(shared_memory_name_, motor_controls_map_name_,
                         motor_controls_map_);
    }

    // we do not send the command if the thread is asked to stopped
    if(!is_hardware_communication_stopped() && ros::ok()) {
      // send the command to the motors
      set_motor_controls_from_map(motor_controls_map_);
    }
    hwc_timer_.tac();
  }
  // We use this function here because the loop might stop because of ROS
  // and we need the flag to be set to off
  stop_hardware_communication();

  std::cout << "HARDWARE: Dump active time measurement" << std::endl;
  hwc_active_timer_.dump_measurements(hwc_active_timer_file_);
  std::cout << "HARDWARE: sleep time measurement" << std::endl;
  hwc_sleep_timer_.dump_measurements(hwc_sleep_timer_file_);
  std::cout << "HARDWARE: hwc time measurement" << std::endl;
  hwc_timer_.dump_measurements(hwc_timer_file_);
  cond_var_->unlock_scope();
  std::cout << "HARDWARE: Stop loop" << std::endl;
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

void* DynamicGraphManager::single_process_real_time_loop()
{
  // //std::cout << "DG: Locking scope..." << std::endl;
  std::cout << "DG: Start loop" << std::endl;
  while(!is_dynamic_graph_stopped() && ros::ok())
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
  //printf("dynamic graph thread stopped\n");
  std::cout << "DG: Stop loop" << std::endl;
}
