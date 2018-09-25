/**
 * \file dynamic_graph_manager.cpp
 * \brief The Control Manager
 * \author Maximilien Naveau
 * \date 2018
 *
 * Implementation of the DynamicGraphManager class
 *
 */

#include <dynamic_graph_manager/ros_init.hh>
#include <dynamic_graph_manager/dynamic_graph_manager.hh>

using namespace dynamic_graph;

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

  missed_control_count_ = 0;
  max_missed_control_ = 0;
  hardware_communication_sleep_time_usec_ = 0;
}

DynamicGraphManager::~DynamicGraphManager()
{
  // kill all ros related stuff
  ros_service_start_dg_.shutdown();
  ros_service_stop_dg_.shutdown();
  ros_shutdown();
  // wait for the dynamic graph thread to stop
  stop_dynamic_graph();
  wait_stop_dynamic_graph();
  // wait for the hardware communication thread to stop
  stop_hardware_communication();
  wait_stop_hardware_communication();
  // clean the shared memory
  shared_memory::clear_shared_memory(shared_memory_name_);
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
  hardware_communication_sleep_time_usec_ =
      params_["hardware_communication"]
      ["hardware_communication_sleep_time_usec"].as<unsigned>();

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
    usleep(1000);
  }
  if(thread_dynamic_graph_)
  {
    real_time_tools::join_thread(*thread_dynamic_graph_);
  }
}

void DynamicGraphManager::wait_stop_hardware_communication()
{
  while(!is_hardware_communication_stopped())
  {
    usleep(1000);
  }
  if(thread_hardware_communication_)
  {
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
  ros::NodeHandle& ros_node_handle = ros_init("dynamic_graph");
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
  ros_init("hardware_communication");

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
  //std::cout << "DG: Locking scope..." << std::endl;
  cond_var_->lock_scope();
  std::cout << "DG: Start loop" << std::endl;
  while(!is_dynamic_graph_stopped() && ros::ok())
  {
    // wait that the hardware_communication process acquiers the data
    cond_var_->wait();

    // read the sensor from the shared memory
    shared_memory::get(shared_memory_name_, sensors_map_name_, sensors_map_);

    // call the dynamic graph
    device_->set_sensors_from_map(sensors_map_);
    device_->execute_graph();
    device_->get_controls_to_map(motor_controls_map_);

    // write the command to the shared memory
    shared_memory::set(shared_memory_name_, motor_controls_map_name_,
                       motor_controls_map_);

    //std::cout << "DG: notifies" << std::endl;
    // notify the hardware_communication process that the control has been done
    cond_var_->notify_all();
  }
  // we use this function here because the loop might stop because of ROS
  stop_dynamic_graph();
  //printf("dynamic graph thread stopped\n");
  std::cout << "DG: Stop loop" << std::endl;
  cond_var_->unlock_scope();
}

void* DynamicGraphManager::hardware_communication_real_time_loop()
{
  //std::cout << "HARDWARE: Locking scope..." << std::endl;
  cond_var_->lock_scope();
  assert(!is_hardware_communication_stopped_ && "The loop is started");
  assert(ros::ok() && "Ros has to be initialized");
  // printf("Start hardware communication loop");
  std::cout << "HARDWARE: Start loop" << std::endl;
  while(!is_hardware_communication_stopped() && ros::ok())
  {
    // call the sensors
    get_sensors_to_map(sensors_map_);

    // write the sensors to the shared memory
    shared_memory::set(shared_memory_name_, sensors_map_name_, sensors_map_);

    // notify the dynamic graph process that is can compute the graph
    cond_var_->notify_all();

    if(is_real_robot_)
    {
      // sleep has much has we can TODO: find a better way to compute the sleep
      // time, for now it is 800us
      //std::cout << "HARDWARE: Waiting..." << std::endl;
      if(cond_var_->timed_wait(hardware_communication_sleep_time_usec_))
      {
        // this thread has been awaken by the dynamic graph.
        missed_control_count_ = 0;
      }else{
        // this thread has automatically awaken
        ++missed_control_count_;
      }

      if(is_in_safety_mode())
      {
        compute_safety_controls();
        // we write the safety command in the shared memory in order to perform
        // some interpolation
        shared_memory::set(shared_memory_name_, motor_controls_map_name_,
                           motor_controls_map_);
      }else{
        // we read the command from the shared memory
        shared_memory::get(shared_memory_name_, motor_controls_map_name_,
                           motor_controls_map_);
      }
    }
    else
    {
      cond_var_->wait();
    }

    // send the command to the motors
    set_motor_controls_from_map(motor_controls_map_);
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
