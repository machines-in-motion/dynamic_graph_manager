/**
 * @file test_dynamic_graph_manager.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 * @date 2019-05-22
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sys/types.h>
#include <signal.h>
#include <thread>

// We use another class of this package which has an independant unittest.
#include "dynamic_graph_manager/ros_init.hh"

// The class to test
#include "dynamic_graph_manager/dynamic_graph_manager.hh"

// Service for the user command unittest
#include "dynamic_graph_manager/TestUserCmdBool.h"

/**
 * SETUP THE TESTING CLASS
 */

/**
 * @brief The DISABLED_TestDynamicGraphManager class is used to disable test
 */
class DISABLED_TestDynamicGraphManager : public ::testing::Test
{
public:
  YAML::Node params_;
};

/**
 * @brief The TestDynamicGraphManager class: test suit template for setting up
 * the unit tests for the DynamicGraphManager
 */
class TestDynamicGraphManager : public ::testing::Test {
protected:
  /**
   * @brief SetUp, is executed before the unit tests
   */
  void SetUp() {
    params_ = YAML::LoadFile(TEST_CONFIG_PATH +
                             std::string("simple_robot.yaml"));
    shared_memory::clear_shared_memory(
      dynamic_graph::DynamicGraphManager::shared_memory_name_);
  }

  /**
   * @brief TearDown, is executed after the unit tests
   */
  void TearDown() {
    dynamic_graph::ros_shutdown();
  }

  /**
   * @brief Params of the DGM
   */
  YAML::Node params_;
};

/**
 * @brief This class is a simple dynamic graph manager with a fake hardware
 * interface used for unittesting
 */
class SimpleDGM : public dynamic_graph::DynamicGraphManager
{
public:
  /**
   * @brief Construct a new SimpleDGM object
   */
  SimpleDGM(): dynamic_graph::DynamicGraphManager(){
    boolean_set_by_user_cmd_ = false;
  }
  /**
   * @brief Destroy the SimpleDGM object
   */
  ~SimpleDGM(){}
  /**
   * @brief Simple overload doing nothing. We have no hardware here for the unit
   * tests.
   */
  void initialize_hardware_communication_process()
  {
    // get the hardware communication ros node handle
    ros::NodeHandle& ros_node_handle = dynamic_graph::ros_init(
      dynamic_graph::DynamicGraphManager::hw_com_ros_node_name_);
    /** initialize the user commands */
    ros_user_commands_.push_back(ros_node_handle.advertiseService(
        "set_a_boolean", &SimpleDGM::user_command_callback, this));
  }
  /**
   * @brief Get the sensors to the map object.
   * 
   * @param map of sensors
   */
  void get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    map["encoders"].setRandom();
    map["imu_accelerometer"].setRandom();
    map["imu_gyroscope"].setRandom();
    map["imu"].setRandom();
  }
  /**
   * @brief Set the motor controls from map object to no hardware. So nothing to
   * be done here
   * 
   * @param map of controls
   */
  void set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map)
  {/*Nothing to be done*/}

  /**
   * @brief Get the has_user_command_been_executed_ object
   */
  bool get_has_user_command_been_executed()
  {
    // Here we nee to protect the access to this ressource as it may conflict
    // with different thread. Please use cond_var_ to make sure everything
    // is access on its due time.
    bool ret = boolean_set_by_user_cmd_;
    return ret;
  }

  /**
   * @brief This service callback parse the ros messages and register and
   * internal method for further call using the data from the ros message.
   * 
   * @param req this is the user argument
   * @param res this is the feedback of the user command
   * @return true in case the service hase been properly executed
   * @return false in case of failure
   */
  bool user_command_callback(dynamic_graph_manager::TestUserCmdBool::Request& req,
                             dynamic_graph_manager::TestUserCmdBool::Response& res)
  {
    // parse and register the command for further call.
    add_user_command(std::bind(&SimpleDGM::user_command, 
                     this, req.input_boolean));
    // return whatever the user want
    res.sanity_check = true;
    
    // the service has been executed properly
    return true;
  }

private:
  /**
   * @brief The actuall user command called in the real time thread
   * 
   * @param user_input is some boolean
   */
  void user_command(bool user_input)
  {
    // do something with the internal state or with the hardware
    boolean_set_by_user_cmd_ = user_input;
  }
  // some internal hardware class or obect. Here just a simple boolean for
  // unit testing
  std::atomic_bool boolean_set_by_user_cmd_;
};

/**
 * @brief The TestDynamicGraphManagerHWC class: test suit template for setting
 * up the unit tests for the DynamicGraphManager hardware communication user
 * commands
 */
class TestDynamicGraphManagerHWC : public ::testing::Test {
protected:
  /**
   * @brief SetUp, is executed before the unit tests
   */
  void SetUp() {
    params_ = YAML::LoadFile(TEST_CONFIG_PATH +
                             std::string("simple_robot.yaml"));
    shared_memory::clear_shared_memory(
      dynamic_graph::DynamicGraphManager::shared_memory_name_);
  }

  /**
   * @brief TearDown, is executed after the unit tests
   */
  void TearDown() {
    dgm_->stop_hardware_communication();
    dgm_->wait_stop_hardware_communication();
    dynamic_graph::ros_shutdown();
  }

  /**
   * @brief Params of the DGM
   */
  YAML::Node params_;

  /**
   * @brief a pointer for the test
   */
  std::unique_ptr<SimpleDGM> dgm_;
};

bool start_user_cmd_ros_service(bool user_input)
{ 
  // wait a bit
  sleep(1.0);
  // service name
  std::string service_name = "/hardware_communication/set_a_boolean";
  // Create a client from a temporary node
  ros::service::waitForService(service_name);
  // fill the command message
  dynamic_graph_manager::TestUserCmdBool srv;
  srv.request.input_boolean = user_input;
  srv.response.sanity_check = false;
  // call the service
  bool ret = ros::service::call(service_name, srv);
  // wait a bit
  sleep(1.0);
  // return the sanity check
  return srv.response.sanity_check && ret;
}

void start_dg_ros_service()
{ 
  // wait a bit
  sleep(0.05);
  // service name
  std::string service_name = "/dynamic_graph/start_dynamic_graph";
  // Create a client from a temporary node
  ros::service::waitForService(service_name, true);
  // fill the command message
  std_srvs::Empty srv;
  // call the service
  ros::service::call(service_name, srv);
}

void stop_dg_ros_service()
{ 
  // service name
  std::string service_name = "/dynamic_graph/stop_dynamic_graph";
  // Create a client from a temporary node
  ros::service::waitForService(service_name, true);
  // fill the command message
  std_srvs::Empty srv;
  // call the service
  ros::service::call(service_name, srv); 
}

void start_run_python_command_ros_service(std::string cmd)
{ 
  // service name
  std::string service_name = "/dynamic_graph/run_python_command";
  // wait for the service
  ros::service::waitForService(service_name, true);
  // fill the command message
  dynamic_graph_manager::RunCommand run_com_msg;
  run_com_msg.request.input = cmd;
  // Call the server
  ros::service::call(service_name, run_com_msg);
  
  // Check if the call went well
  // std::cerr << "python cmd: \"" << run_com_msg.request.input << "\"" << std::endl;
  // std::cerr << "python response: \"" << run_com_msg.response.result << "\""  << std::endl;
  // std::cerr << "python stdout: \"" << run_com_msg.response.result << "\""  << std::endl;
  // std::cerr << "python stderr: \"" << run_com_msg.response.result << "\""  << std::endl;
}

void start_run_python_script_ros_service(std::string file_name)
{ 
  // service name
  std::string service_name = "/dynamic_graph/run_python_script";
  // wait for the service
  ros::service::waitForService(service_name, true);
  // fill the command message
  dynamic_graph_manager::RunPythonFile run_file_msg;
  run_file_msg.request.input = file_name;
  // Call the server
  ros::service::call(service_name, run_file_msg);
}






/*****************
 * Start Testing *
 *****************/

/**
 * Here we check that the constructor throw no error
 */
TEST_F(TestDynamicGraphManager, test_constructor)
{
  /** Test */
  ASSERT_NO_THROW(
    dynamic_graph::DynamicGraphManager dgm;
  );
}

/**
 * Here we check that the destructor throw no error
 */
TEST_F(TestDynamicGraphManager, test_destructor)
{
  /** Test */
  ASSERT_NO_THROW(
    {
      dynamic_graph::DynamicGraphManager dgm;
    }
  );
}

/**
 * Here we check that the initialization throw no error
 */
TEST_F(TestDynamicGraphManager, test_initialize)
{
  /** Test */
  ASSERT_NO_THROW(
    {
      dynamic_graph::DynamicGraphManager dgm;
      dgm.initialize(params_);
    }
  );
}

/**
 * Here we check that the initialization does not create nor initialize a ros
 * node.
 */
TEST_F(TestDynamicGraphManager, test_initialize_no_dg_node_handle)
{
  /** Setup */
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);

  /** Test */
  ASSERT_FALSE(dynamic_graph::ros_exist(dgm.dg_ros_node_name_));
}

/**
 * Here we check that the initialization does not create nor initialize a ros
 * node.
 */
TEST_F(TestDynamicGraphManager, test_initialize_no_hwc_node_handle)
{
  /** Setup */
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);

  /** Test */
  ASSERT_FALSE(dynamic_graph::ros_exist(dgm.hw_com_ros_node_name_));
}

/**
 * Here we check that the initialization of the dynamic graph process creates
 * the ros node properly.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_node_handle_exists)
{
  /** Setup */
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();

  /** Test */
  ASSERT_TRUE(dynamic_graph::ros_exist(dgm.dg_ros_node_name_));
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. The "/dynamic_graph/start_dynamic_graph" ros sservice must be
 * available by now.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_start_dg_exist)
{
  /** Setup */
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();
  ros::NodeHandle& n = dynamic_graph::ros_init(dgm.dg_ros_node_name_);

  /** Test */
  ASSERT_TRUE(ros::service::exists("/dynamic_graph/start_dynamic_graph", true));
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. The "/dynamic_graph/stop_dynamic_graph" ros sservice must be
 * available by now.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_stop_dg_exist)
{
  /** Setup */
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();
  ros::NodeHandle& n = dynamic_graph::ros_init(dgm.dg_ros_node_name_);

  /** Test */
  ASSERT_TRUE(ros::service::exists("/dynamic_graph/stop_dynamic_graph", true));
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. The "/dynamic_graph/run_python_command" ros sservice must be
 * available by now.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_run_python_cmd_exist)
{
  /** Setup */
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();
  ros::NodeHandle& n = dynamic_graph::ros_init(dgm.dg_ros_node_name_);

  /** Test */
  ASSERT_TRUE(ros::service::exists("/dynamic_graph/run_python_command", true));
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. The "/dynamic_graph/run_python_script" ros sservice must be
 * available by now.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_run_python_file_exist)
{
  /** Setup */
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();
  ros::NodeHandle& n = dynamic_graph::ros_init(dgm.dg_ros_node_name_);

  /** Test */
  ASSERT_TRUE(ros::service::exists("/dynamic_graph/run_python_script", true));
}

/**
 * Here we check that the initialization of the hardware communication fails
 * if we call the methode from the raw dynamic graph manager class.
 */
TEST_F(TestDynamicGraphManager, test_initialize_hwc_throw)
{
  /** Setup */
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);

  /** Test */
  ASSERT_THROW(
      dgm.initialize_hardware_communication_process(), std::runtime_error
  );
}

/**
 * Here we check that the initialization of the hardware communication runs
 * smoothly using a daughter class of the dynamic graph manager.
 */
TEST_F(TestDynamicGraphManager, test_initialize_hwc_no_throw)
{
  /** Setup */
  SimpleDGM dgm;
  dgm.initialize(params_);

  /** Test */
  ASSERT_NO_THROW(
      dgm.initialize_hardware_communication_process();
  );
}

/**
 * Here we check that the initialization of the hardware communication runs
 * smoothly using a daughter class of the dynamic graph manager and that a ros
 * node is NOT created. The ros node is created upon the launch of the thread.
 * During a common initilialization phase.
 */
TEST_F(TestDynamicGraphManager, test_hwc_process_node_handle_exists)
{
  /** Setup */
  SimpleDGM dgm;
  dgm.initialize(params_);
  dgm.initialize_hardware_communication_process();

  /** Test */
  ASSERT_TRUE(dynamic_graph::ros_exist(dgm.hw_com_ros_node_name_));
}

/**
 * Here we check that upon initialization, the hardware communication thread is
 * not running
 */
TEST_F(TestDynamicGraphManager, test_hwc_process_stopped_on_init)
{
  /** Setup */
  SimpleDGM dgm;
  dgm.initialize(params_);
  dgm.initialize_hardware_communication_process();

  /** Test */
  ASSERT_TRUE(dgm.is_hardware_communication_stopped());
}


/**
 * Check that the dynamic graph is stopped after initialization.
 */
TEST_F(TestDynamicGraphManager, test_dynamic_graph_stopped_upon_start)
{
  /** Setup */
  // Create a dynamic_graph_manager (DGM)
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  // Initialize dg process
  dgm.initialize_dynamic_graph_process();

  /** Test */
  ASSERT_TRUE(dgm.is_dynamic_graph_stopped());  
}

/**
 * Here we check that if the "/dynamic_graph/start_dynamic_graph" ros service
 * works properly.
 */
TEST_F(TestDynamicGraphManager, test_start_dynamic_graph_ros_services)
{
  /** Setup */
  // Create a dynamic_graph_manager (DGM)
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  // Initialize dg process
  dgm.initialize_dynamic_graph_process();
  // Create the service message
  std_srvs::Empty srv;
  // Create a client from a temporary node
  start_dg_ros_service();
  
  /** Test */
  ASSERT_FALSE(dgm.is_dynamic_graph_stopped());
}

/**
 * Here we check that if the "/dynamic_graph/stop_dynamic_graph" ros service
 * works properly.
 */
TEST_F(TestDynamicGraphManager, test_stop_dynamic_graph_ros_services)
{
  /** Setup */
  // Create a dynamic_graph_manager (DGM)
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  // Initialize dg process
  dgm.initialize_dynamic_graph_process();
  // Create the service message
  std_srvs::Empty srv;
  // Start the dynamic graph
  dgm.start_dynamic_graph();
  // Create a client from a temporary node
  stop_dg_ros_service();
  
  /** Test */
  ASSERT_TRUE(dgm.is_dynamic_graph_stopped());
}

/**
 * Here we check that if the method start_dynamic_graph works properly.
 */
TEST_F(TestDynamicGraphManager, test_start_dynamic_graph_methods)
{
  /** Setup */
  // Create a dynamic_graph_manager (DGM)
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  // Initialize dg process
  dgm.initialize_dynamic_graph_process();
  // Start the dynamic graph
  dgm.start_dynamic_graph();
  
  /** Test */
  ASSERT_FALSE(dgm.is_dynamic_graph_stopped());
}

/**
 * Here we check that if the method stop_dynamic_graph works properly.
 */
TEST_F(TestDynamicGraphManager, test_stop_dynamic_graph_methods)
{
  /** Setup */
  // Create a dynamic_graph_manager (DGM)
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  // Initialize dg process
  dgm.initialize_dynamic_graph_process();
  // Start the dynamic graph
  dgm.start_dynamic_graph();
  // Stop it again :)
  dgm.stop_dynamic_graph();
  
  /** Test */
  ASSERT_TRUE(dgm.is_dynamic_graph_stopped());
}

void start_dg_method(dynamic_graph::DynamicGraphManager* dgm)
{
  sleep(0.05);
  dgm->start_dynamic_graph();
}

/**
 * @brief Test the methods that put the current thread to sleep until the
 * method "start_dynamic_graph" is called.
 */
TEST_F(TestDynamicGraphManager, test_wait_start_dynamic_graph_method)
{
  /** Setup */
  // first we create the dynamic graph process
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();
  // then we launch a thread that will call the start dynamic graph
  std::thread start_dg_thread = std::thread(&start_dg_method, &dgm);
  // Pause the current thread
  dgm.wait_start_dynamic_graph();
  // Wait until the thread is finished
  start_dg_thread.join();

  /** Test */
  ASSERT_FALSE(dgm.is_dynamic_graph_stopped());
}

/**
 * @brief Test the methods that put the current thread to sleep until the
 * rosservice "dynamic_graph/start_dynamic_graph" is called.
 */
TEST_F(TestDynamicGraphManager, test_wait_start_dynamic_graph_rosservice)
{
  /** Setup */
  // first we create the dynamic graph process
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();
  // then we launch a thread that will call the start dynamic graph
  std::thread start_dg_thread = std::thread(&start_dg_ros_service);
  // Pause the current thread
  dgm.wait_start_dynamic_graph();
  // Wait until the thread is finished
  start_dg_thread.join();

  /** Test */
  ASSERT_FALSE(dgm.is_dynamic_graph_stopped());
}

/**
 * @brief Call start and stop dynamic graph from methods
 * 
 * @param a dynamic graph manager 
 */
void manage_dg_process(dynamic_graph::DynamicGraphManager* dgm)
{ 
  // wait a bit so the main thread is executed
  sleep(1.0);

  // start the dynamic graph.
  dgm->start_dynamic_graph();

  // Sleep a bit so the thread is launched and can spin once
  sleep(2);

  // stop the dynamic graph
  dgm->stop_dynamic_graph();
}

/**
 * This test runs the dynamic graph process completely
 */
TEST_F(TestDynamicGraphManager, test_run_dynamic_graph_process)
{
  /** Setup */

  // Initialize the dynamic graph manager
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();

  // Modify the ouput torques
  dynamicgraph::Vector vec;
  vec.resize(5);
  vec.fill(0.0);
  vec(0) = 1.0;
  dgm.device().motor_controls_in_.at("torques")->setConstant(vec);

  // then we launch a thread that will call the start dynamic graph
  std::thread start_dg_thread = std::thread(&manage_dg_process, &dgm);
  // launch the process and wait it to start
  dgm.run_dynamic_graph_process();
  // Once started, wait for it to stop
  dgm.wait_stop_dynamic_graph();
  // join the thread
  start_dg_thread.join();

  /** Test */
  double norm_torques = dgm.device().motor_controls_map_.at("torques").norm();
  ASSERT_DOUBLE_EQ(norm_torques, 1.0);
}

/**
 * @brief Test if the start and stop of the hardware communication thread
 * is done nicely
 */
TEST_F(TestDynamicGraphManager, test_run_hardware_communication_process)
{
  /** Setup */
  SimpleDGM dgm;
  dgm.initialize(params_);
  dgm.initialize_hardware_communication_process();
  dgm.run_hardware_communication_process();
  usleep(5000);
  
  /** Test */
  ASSERT_FALSE(dgm.is_hardware_communication_stopped());
  
  /** Some other setup */
  dgm.stop_hardware_communication();
  dgm.wait_stop_hardware_communication();
  
  /** Final test */
  ASSERT_TRUE(dgm.is_hardware_communication_stopped());
}

/**
 * @brief Test if the start and stop of the hardware communication thread
 * is done nicely
 */
TEST_F(TestDynamicGraphManagerHWC, test_run_user_cmd)
{
  /** Setup */
  ros::NodeHandle& node = dynamic_graph::ros_init(
    dynamic_graph::DynamicGraphManager::hw_com_ros_node_name_);
  dgm_.reset(new SimpleDGM());
  dgm_->initialize(params_);
  dgm_->initialize_hardware_communication_process();
  dgm_->run_hardware_communication_process();
  
  /** Test 1 */
  ASSERT_FALSE(dgm_->get_has_user_command_been_executed());

  /** Test 2 */
  ASSERT_TRUE(start_user_cmd_ros_service(true));
  ASSERT_TRUE(dgm_->get_has_user_command_been_executed());
  
  /** Test 3 */
  ASSERT_TRUE(start_user_cmd_ros_service(false));
  ASSERT_FALSE(dgm_->get_has_user_command_been_executed());

  /** Tear down */
  // see TestDynamicGraphManagerHWC::TearDown
}

/**
 * @brief Test the standard behavior of the DGM until the safety mode is
 * activated
 */
TEST_F(DISABLED_TestDynamicGraphManager, DISABLED_test_complete_run_with_safety_mode_activation)
{
  /** Setup */
  std::cerr << ("main: Setup") << std::endl;
  SimpleDGM dgm;
  dgm.initialize(params_);
  dgm.run();
  // wait that everything started
  usleep(5000);
  // Start the dynamic graph
  start_dg_ros_service();
  std::cerr << ("main: Dynamic Graph Started") << std::endl;
  // wait few iteration of the dynamic_graph
  sleep(1);
  // Stop the dynamic graph
  stop_dg_ros_service();
  std::cerr << ("main: Dynamic Graph Stopped") << std::endl;
  std::cerr << ("main: Wait for the DG process to die") << std::endl;
  kill(dgm.pid_dynamic_graph_process(), SIGKILL);
  while(!dgm.has_dynamic_graph_process_died())
  {
    usleep(1000);
  }
  std::cerr << ("main: Dynamic Graph Process Died") << std::endl;
  std::cerr << ("main: Check that the security mode has been activated")
            << std::endl;

  /** Test */
  ASSERT_TRUE(dgm.is_in_safety_mode());
  
  
  /** Tear Down */
  dgm.stop_hardware_communication();
  usleep(5000);
}

/**
 * WARNING this has to be the last test as we ask for ros::shutdown
 * Test if ros::shutdown is actually killing the thread
 */
TEST_F(DISABLED_TestDynamicGraphManager, DISABLED_test_hwc_process_ros_shutdown)
{
  /** Setup */
  SimpleDGM dgm;
  dgm.initialize(params_);
  dgm.initialize_hardware_communication_process();
  dgm.run_hardware_communication_process();
  usleep(5000);

  /** First test */
  ASSERT_FALSE(dgm.is_hardware_communication_stopped());

  /** Some more setup */
  ros::requestShutdown();
  dgm.wait_stop_hardware_communication();

  /** Final test */
  ASSERT_TRUE(dgm.is_hardware_communication_stopped());
}


















/**
 * WARNING! IN ROSTEST YOU NEED TO ADD THE "DISABLED_" also in the test name
 * to be able to notice a disabled test.
 */

/**
 * Test that we can stop and reboot the dynamic graph process.
 */
TEST_F(DISABLED_TestDynamicGraphManager, DISABLED_test_dynamic_graph_re_initialization)
{
  // verify that ros is not active
  ASSERT_TRUE(!ros::ok());

  // initialize ROS (not needed here as the dynamic graph manager does it)

  // create the DGM
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);

  ASSERT_FALSE(ros::service::exists(
                 "/dynamic_graph/start_dynamic_graph", false));
  ASSERT_FALSE(ros::service::exists(
                 "/dynamic_graph/stop_dynamic_graph", false));
  ASSERT_FALSE(ros::service::exists(
                 "/dynamic_graph/run_python_command", false));
  ASSERT_FALSE(ros::service::exists(
                 "/dynamic_graph/run_python_script", false));

  // initialize dgm
  dgm.initialize_dynamic_graph_process();

  ASSERT_TRUE(ros::service::exists(
                 "/dynamic_graph/start_dynamic_graph", false));
  ASSERT_TRUE(ros::service::exists(
                 "/dynamic_graph/stop_dynamic_graph", false));
  ASSERT_TRUE(ros::service::exists(
                 "/dynamic_graph/run_python_command", false));
  ASSERT_TRUE(ros::service::exists(
                 "/dynamic_graph/run_python_script", false));

  // Check that ros is active
  ASSERT_TRUE(ros::ok());

  // check the run_python_command service
  ros::NodeHandle node_handle("test");
  dynamic_graph_manager::RunCommand run_com_msg;
  run_com_msg.request.input = "1 + 1";
  ros::ServiceClient run_python_command_client =
      node_handle.serviceClient<dynamic_graph_manager::RunCommand>(
        "/dynamic_graph/run_python_command");
  ASSERT_TRUE(run_python_command_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(run_python_command_client.call(run_com_msg));
  ASSERT_EQ(run_com_msg.response.result, "2");
  std::cout << ("The run_python_command service has"
                " been called successfully") << std::endl;

  // check the run_script service and the python interpreter
  dynamic_graph_manager::RunPythonFile run_file_msg;
  run_file_msg.request.input = TEST_CONFIG_PATH + std::string("simple_add.py");
  ros::ServiceClient run_script_client =
      node_handle.serviceClient<dynamic_graph_manager::RunPythonFile>(
        "/dynamic_graph/run_python_script");
  ASSERT_TRUE(run_script_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(run_script_client.call(run_file_msg));
  ASSERT_EQ(run_file_msg.response.result, "File parsed");
  std::cout << ("The run_script service has"
                " been called successfully") << std::endl;

  // check that "a" exists and has a value "3"
  run_com_msg.request.input = "a";
  ASSERT_TRUE(run_python_command_client.call(run_com_msg));
  ASSERT_EQ(run_com_msg.response.result, "3");

  dgm.initialize_dynamic_graph_process();

  // reset the client
  run_python_command_client =
        node_handle.serviceClient<dynamic_graph_manager::RunCommand>(
          "/dynamic_graph/run_python_command");

//  // perform a simple operation
//  run_com_msg.request.input = "1 + 1";
//  run_python_command_client.call(run_com_msg);
//  ASSERT_EQ(run_com_msg.response.result, "False");

//  // check that "a" does not exists
//  run_com_msg.request.input = "'a' in globals()";
//  run_python_command_client.call(run_com_msg);
//  ASSERT_EQ(run_com_msg.response.result, "False");

//  run_com_msg.request.input = "'a' in locals()";
//  run_python_command_client.call(run_com_msg);
//  ASSERT_EQ(run_com_msg.response.result, "False");

}