/**
 * \file test_dynamic_graph_manager.cpp
 * \brief DynamicGraphManager unit tests
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file implements a suit of unit tests for the DynamicGraphManager class.
 * @see https://git-amd.tuebingen.mpg.de/amd-clmc/ci_example/wikis/catkin:-how-to-implement-unit-tests
 */

#include <gtest/gtest.h>
#include <shared_memory/shared_memory.hpp>
#include <dynamic_graph_manager/ros_init.hh>
#include <dynamic_graph_manager/dynamic_graph_manager.hh>
#include <ros/ros.h>
#include <sys/types.h>
#include <signal.h>

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
    shared_memory::clear_shared_memory("DGM_ShM");
  }

  /**
   * @brief TearDown, is executed after teh unit tests
   */
  void TearDown() {
    dynamic_graph::ros_shutdown();
  }

  YAML::Node params_;
};

/**
 * @brief This class is a simple dynamic graph manager with a fake hardware
 * interface used for unittesting
 */
class SimpleDGM : public dynamic_graph::DynamicGraphManager
{
public:
  SimpleDGM(): dynamic_graph::DynamicGraphManager(){}
  ~SimpleDGM(){}
  void initialize_hardware_communication_process()
  {/*Nothing to be done*/}
  void get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    map["encoders"].setRandom();
    map["imu_accelerometer"].setRandom();
    map["imu_gyroscope"].setRandom();
    map["imu"].setRandom();
  }
  void set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map)
  {/*Nothing to be done*/}
};









/*****************
 * Start Testing *
 *****************/

/**
 * Here we check that the constructor throw no error
 */
TEST_F(TestDynamicGraphManager, test_constructor)
{
  ASSERT_NO_THROW(
    dynamic_graph::DynamicGraphManager dgm;
  );
}

/**
 * Here we check that the destructor throw no error
 */
TEST_F(TestDynamicGraphManager, test_destructor)
{
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
  ASSERT_NO_THROW(
    {
      dynamic_graph::DynamicGraphManager dgm;
      dgm.initialize(params_);
    }
  );
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. 4 ros services must be available by now. By test that they
 * are initialized properly.
 */
TEST_F(TestDynamicGraphManager, test_initialize_dynamic_graph_process)
{
  // Setup
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();
  ros::NodeHandle& n = dynamic_graph::ros_init(dgm.dg_ros_node_name_);
  ros::ServiceClient start_dg =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph/start_dynamic_graph");
  ros::ServiceClient stop_dg =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph/stop_dynamic_graph");
  ros::ServiceClient run_py_cmd =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph/run_python_command");
  ros::ServiceClient run_py_script =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph/run_python_script");

  // Assert
  ASSERT_TRUE(start_dg.isValid());
  ASSERT_TRUE(stop_dg.isValid());
  ASSERT_TRUE(run_py_cmd.isValid());
  ASSERT_TRUE(run_py_script.isValid());

  // Tear down
}

/**
 * Here we check that the initialization of the hardware communication fails
 * if we call the methode from the raw dynamic graph manager class.
 */
TEST_F(TestDynamicGraphManager, test_initialize_hwc_throw)
{
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
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
  SimpleDGM dgm;
  dgm.initialize(params_);
  ASSERT_NO_THROW(
      dgm.initialize_hardware_communication_process();
  );
}

/**
 * Here we check that if the start or stop ros services are working properly.
 */
TEST_F(TestDynamicGraphManager, test_start_stop_ros_services)
{
  // verify that the ros node does not exists.
  ASSERT_FALSE(dynamic_graph::ros_exist(
    dynamic_graph::DynamicGraphManager::dg_ros_node_name_));

  // Create a dynamic_graph_manager (DGM)
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);

  // Initialize dgm
  dgm.initialize_dynamic_graph_process();

  // Verify that the ros node has been created.
  ASSERT_TRUE(dynamic_graph::ros_exist(
    dynamic_graph::DynamicGraphManager::dg_ros_node_name_));

  // Check that the dynamic graph is stopped
  ASSERT_TRUE(dgm.is_dynamic_graph_stopped());  
  
  // create clients the start and stop dynamic graph service from the DGM
  std_srvs::Empty srv;

  // Check that the start dynamic graph service works as expected
  ros::NodeHandle& n = dynamic_graph::ros_init(dgm.dg_ros_node_name_);
  ros::ServiceClient start_dynamic_graph_client =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph/start_dynamic_graph");
  ASSERT_TRUE(start_dynamic_graph_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(start_dynamic_graph_client.call(srv));
  ASSERT_TRUE(!dgm.is_dynamic_graph_stopped());
  std::cout << ("The start_dynamic_graph service has"
                " been called successfully") << std::endl;

  // Check that the stop dynamic graph service works as expected
  ros::ServiceClient stop_dynamic_graph_client =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph/stop_dynamic_graph");
  ASSERT_TRUE(stop_dynamic_graph_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(stop_dynamic_graph_client.call(srv));
  ASSERT_TRUE(dgm.is_dynamic_graph_stopped());
  std::cout << ("The stop_dynamic_graph service has"
                " been called successfully") << std::endl;
}

/**
 * Here we check that if the communication with the python interpretor is
 * working correctly.
 */
TEST_F(TestDynamicGraphManager, test_python_interpreter)
{
  // Verify that the ros node does not exists.
  ASSERT_FALSE(dynamic_graph::ros_exist(
    dynamic_graph::DynamicGraphManager::dg_ros_node_name_));

  // Initialize ROS, create an asynchronous spinner and get a node handle
  ros::NodeHandle& node_handle = dynamic_graph::ros_init (
    dynamic_graph::DynamicGraphManager::dg_ros_node_name_);

  // Verify that the ros node has been created.
  ASSERT_TRUE(dynamic_graph::ros_exist(
    dynamic_graph::DynamicGraphManager::dg_ros_node_name_));  

  // Create the python interpreter
  dynamic_graph::RosPythonInterpreter rpi (node_handle);

  // Advertize the ros::service
  rpi.start_ros_service();

  // Check the run_python_command service
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
}