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
#include <dynamic_graph_manager/ros_init.hh>
#include <dynamic_graph_manager/dynamic_graph_manager.hh>
#include <ros/ros.h>

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
    params_ = YAML::LoadFile(TEST_CONFIG_FOLDER +
                             std::string("simple_robot.yaml"));
  }

  /**
   * @brief TearDown, is executed after teh unit tests
   */
  void TearDown() {
    if(ros::ok())
    {
      dynamic_graph::ros_shutdown();
      usleep(500000);
      ros::shutdown();
    }
    assert(!ros::ok() && "ROS must be shut down now.");
  }

  YAML::Node params_;
};


TEST_F(TestDynamicGraphManager, test_constructor)
{
  ASSERT_NO_THROW(
    dynamic_graph::DynamicGraphManager dgm;
  );
}

TEST_F(TestDynamicGraphManager, test_destructor)
{
  ASSERT_NO_THROW(
    {
      dynamic_graph::DynamicGraphManager dgm;
    }
  );
}

TEST_F(TestDynamicGraphManager, test_initialize)
{
  ASSERT_NO_THROW(
    {
      dynamic_graph::DynamicGraphManager dgm;
      dgm.initialize(params_);
    }
  );
}

TEST_F(TestDynamicGraphManager, test_run)
{
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.run();
  usleep(500000);
  dgm.stop_dynamic_graph();
  dgm.stop_hardware_communication();
}

/**
 * @brief test_wait_start_dynamic_graph_fork unstable test.
 */
TEST_F(DISABLED_TestDynamicGraphManager, test_wait_start_dynamic_graph)
{
  ASSERT_TRUE(!ros::ok());
  ros::shutdown();
  pid_t pid = fork();
  if(pid == 0) // Child process
  {
    // std::cout << "Child process started..." << std::endl;
    dynamic_graph::DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();
    ASSERT_TRUE(ros::ok());
    // std::cout << "Child process waiting for the dg to start..." << std::endl;
    dgm.wait_start_dynamic_graph();
    ASSERT_TRUE(!dgm.is_dynamic_graph_stopped());
    // std::cout << "Child process stopped..." << std::endl;
    exit(0);
  }
  else if(pid > 0) // Parent process
  {
    ASSERT_TRUE(!ros::ok());
    ros::NodeHandle& n = dynamic_graph::ros_init();
    ASSERT_TRUE(ros::ok());
    // std::cout << "Parent process started..." << std::endl;
    // create clients the start and stop dynamic graph service from the DGM
    std_srvs::Empty srv;
    // Check that the start dynamic graph service works as expected
    ros::ServiceClient start_dynamic_graph_client =
        n.serviceClient<std_srvs::Empty>(
          "/dynamic_graph_manager/start_dynamic_graph");
    ASSERT_TRUE(start_dynamic_graph_client.waitForExistence());
    ASSERT_TRUE(start_dynamic_graph_client.exists());
    ASSERT_TRUE(start_dynamic_graph_client.call(srv));
    ROS_INFO("The start_dynamic_graph service has been called successfully");
    wait(nullptr);
  }
  else
  {
    ASSERT_TRUE(false && "fork failed");
  }
}

TEST_F(TestDynamicGraphManager, test_initialize_dynamic_graph_process)
{
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();
  ros::NodeHandle& n = dynamic_graph::ros_init();
  ros::ServiceClient start_dg =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph_manager/start_dynamic_graph");
  ros::ServiceClient stop_dg =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph_manager/stop_dynamic_graph");
  ros::ServiceClient run_py_cmd =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph_manager/run_python_command");
  ros::ServiceClient run_py_script =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph_manager/run_python_script");

  ASSERT_TRUE(start_dg.isValid());
  ASSERT_TRUE(stop_dg.isValid());
  ASSERT_TRUE(run_py_cmd.isValid());
  ASSERT_TRUE(run_py_script.isValid());
}

TEST_F(TestDynamicGraphManager, test_run_dynamic_graph_process)
{
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  dgm.initialize_dynamic_graph_process();
  //dgm.run_dynamic_graph_process();

}

TEST_F(TestDynamicGraphManager, test_initialize_hardware_communication_process)
{
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  ASSERT_ANY_THROW(
    dgm.initialize_hardware_communication_process();
  );
}

TEST_F(TestDynamicGraphManager, test_run_hardware_communication_process)
{
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);
  ASSERT_ANY_THROW(
    dgm.run_hardware_communication_process();
  );
}

/**
 * @brief test_start_stop_ros_services, test the start/stop dynamic graph ROS
 * services
 */
TEST_F(TestDynamicGraphManager, test_start_stop_ros_services)
{
  // verify that ros is not active
  ASSERT_TRUE(!ros::ok());

  // initialize ROS (not needed here as the dynamic graph manager does it)

  // Create a dynamic_graph_manager (DGM)
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);

  // initialize dgm
  dgm.initialize_dynamic_graph_process();

  // Check that the dynamic graph is stopped
  ASSERT_TRUE(dgm.is_dynamic_graph_stopped());
  ASSERT_TRUE(ros::isInitialized());

  // create clients the start and stop dynamic graph service from the DGM
  ros::NodeHandle n;
  std_srvs::Empty srv;

  // Check that the start dynamic graph service works as expected
  ros::ServiceClient start_dynamic_graph_client =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph_manager/start_dynamic_graph");
  ASSERT_TRUE(start_dynamic_graph_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(start_dynamic_graph_client.call(srv));
  ASSERT_TRUE(!dgm.is_dynamic_graph_stopped());
  ROS_INFO("The start_dynamic_graph service has been called successfully");

  // Check that the stop dynamic graph service works as expected
  ros::ServiceClient stop_dynamic_graph_client =
      n.serviceClient<std_srvs::Empty>(
        "/dynamic_graph_manager/stop_dynamic_graph");
  ASSERT_TRUE(stop_dynamic_graph_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(stop_dynamic_graph_client.call(srv));
  ASSERT_TRUE(dgm.is_dynamic_graph_stopped());
  ROS_INFO("The stop_dynamic_graph service has been called successfully");
}

TEST_F(TestDynamicGraphManager, test_python_interpreter)
{
  // verify that ros is not active
  ASSERT_TRUE(!ros::ok());

  // initialize ROS, create an asynachronous spinner and get a node handle
  ros::NodeHandle& node_handle = dynamic_graph::ros_init ();

  // Check that ros is active
  ASSERT_TRUE(ros::ok());

  // create the python interpreter
  dynamic_graph::RosPythonInterpreter rpi (node_handle);

  // advertize the ros::service
  rpi.start_ros_service();

  // check the run_python_command service
  dynamic_graph_manager::RunCommand run_com_msg;
  run_com_msg.request.input = "1 + 1";
  ros::ServiceClient run_python_command_client =
      node_handle.serviceClient<dynamic_graph_manager::RunCommand>(
        "/dynamic_graph_manager/run_python_command");
  ASSERT_TRUE(run_python_command_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(run_python_command_client.call(run_com_msg));
  ASSERT_EQ(run_com_msg.response.result, "2");
  ROS_INFO("The run_python_command service has been called successfully");

  // check the run_script service and the python interpreter
  dynamic_graph_manager::RunPythonFile run_file_msg;
  run_file_msg.request.input = TEST_CONFIG_FOLDER + std::string("simple_add.py");
  ros::ServiceClient run_script_client =
      node_handle.serviceClient<dynamic_graph_manager::RunPythonFile>(
        "/dynamic_graph_manager/run_python_script");
  ASSERT_TRUE(run_script_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(run_script_client.call(run_file_msg));
  ASSERT_EQ(run_file_msg.response.result, "File parsed");
  ROS_INFO("The run_script service has been called successfully");

  // check that "a" exists and has a value "3"
  run_com_msg.request.input = "a";
  ASSERT_TRUE(run_python_command_client.call(run_com_msg));
  ASSERT_EQ(run_com_msg.response.result, "3");
}

TEST_F(TestDynamicGraphManager, test_python_interpreter_from_the_DGM)
{
  // verify that ros is not active
  ASSERT_TRUE(!ros::ok());

  // initialize ROS (not needed here as the dynamic graph manager does it)

  // create the DGM
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);

  // initialize dgm
  dgm.initialize_dynamic_graph_process();

  // Check that ros is active
  ASSERT_TRUE(ros::ok());

  // check the run_python_command service
  ros::NodeHandle node_handle("test");
  dynamic_graph_manager::RunCommand run_com_msg;
  run_com_msg.request.input = "1 + 1";
  ros::ServiceClient run_python_command_client =
      node_handle.serviceClient<dynamic_graph_manager::RunCommand>(
        "/dynamic_graph_manager/run_python_command");
  ASSERT_TRUE(run_python_command_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(run_python_command_client.call(run_com_msg));
  ASSERT_EQ(run_com_msg.response.result, "2");
  ROS_INFO("The run_python_command service has been called successfully");

  // check the run_script service and the python interpreter
  dynamic_graph_manager::RunPythonFile run_file_msg;
  run_file_msg.request.input = TEST_CONFIG_FOLDER + std::string("simple_add.py");
  ros::ServiceClient run_script_client =
      node_handle.serviceClient<dynamic_graph_manager::RunPythonFile>(
        "/dynamic_graph_manager/run_python_script");
  ASSERT_TRUE(run_script_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(run_script_client.call(run_file_msg));
  ASSERT_EQ(run_file_msg.response.result, "File parsed");
  ROS_INFO("The run_script service has been called successfully");

  // check that "a" exists and has a value "3"
  run_com_msg.request.input = "a";
  ASSERT_TRUE(run_python_command_client.call(run_com_msg));
  ASSERT_EQ(run_com_msg.response.result, "3");
}

TEST_F(DISABLED_TestDynamicGraphManager, test_dynamic_graph_re_initialization)
{
  // verify that ros is not active
  ASSERT_TRUE(!ros::ok());

  // initialize ROS (not needed here as the dynamic graph manager does it)

  // create the DGM
  dynamic_graph::DynamicGraphManager dgm;
  dgm.initialize(params_);

  ASSERT_FALSE(ros::service::exists(
                 "/dynamic_graph_manager/start_dynamic_graph", false));
  ASSERT_FALSE(ros::service::exists(
                 "/dynamic_graph_manager/stop_dynamic_graph", false));
  ASSERT_FALSE(ros::service::exists(
                 "/dynamic_graph_manager/run_python_command", false));
  ASSERT_FALSE(ros::service::exists(
                 "/dynamic_graph_manager/run_python_script", false));

  // initialize dgm
  dgm.initialize_dynamic_graph_process();

  ASSERT_TRUE(ros::service::exists(
                 "/dynamic_graph_manager/start_dynamic_graph", false));
  ASSERT_TRUE(ros::service::exists(
                 "/dynamic_graph_manager/stop_dynamic_graph", false));
  ASSERT_TRUE(ros::service::exists(
                 "/dynamic_graph_manager/run_python_command", false));
  ASSERT_TRUE(ros::service::exists(
                 "/dynamic_graph_manager/run_python_script", false));

  // Check that ros is active
  ASSERT_TRUE(ros::ok());

  // check the run_python_command service
  ros::NodeHandle node_handle("test");
  dynamic_graph_manager::RunCommand run_com_msg;
  run_com_msg.request.input = "1 + 1";
  ros::ServiceClient run_python_command_client =
      node_handle.serviceClient<dynamic_graph_manager::RunCommand>(
        "/dynamic_graph_manager/run_python_command");
  ASSERT_TRUE(run_python_command_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(run_python_command_client.call(run_com_msg));
  ASSERT_EQ(run_com_msg.response.result, "2");
  ROS_INFO("The run_python_command service has been called successfully");

  // check the run_script service and the python interpreter
  dynamic_graph_manager::RunPythonFile run_file_msg;
  run_file_msg.request.input = TEST_CONFIG_FOLDER + std::string("simple_add.py");
  ros::ServiceClient run_script_client =
      node_handle.serviceClient<dynamic_graph_manager::RunPythonFile>(
        "/dynamic_graph_manager/run_python_script");
  ASSERT_TRUE(run_script_client.waitForExistence(ros::Duration(0.5)));
  ASSERT_TRUE(run_script_client.call(run_file_msg));
  ASSERT_EQ(run_file_msg.response.result, "File parsed");
  ROS_INFO("The run_script service has been called successfully");

  // check that "a" exists and has a value "3"
  run_com_msg.request.input = "a";
  ASSERT_TRUE(run_python_command_client.call(run_com_msg));
  ASSERT_EQ(run_com_msg.response.result, "3");

  dgm.initialize_dynamic_graph_process();

  // reset the client
  run_python_command_client =
        node_handle.serviceClient<dynamic_graph_manager::RunCommand>(
          "/dynamic_graph_manager/run_python_command");

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

TEST_F(TestDynamicGraphManager, test_segfault_fork)
{
  pid_t pid_child = 0;
  pid_t pid_parent = 0;
  pid_t pid = fork();
  if(pid == 0) // Child process
  {
    pid_child = getpid();
    pid_parent = getppid();
    std::cout << "child: pid_child=" << pid_child <<
                 " ; pid_parent=" << pid_parent << std::endl;
    usleep(5000);
    exit(0);
  }
  else if (pid > 0)// Parent process
  {
    pid_child = pid;
    pid_parent = getpid();
    std::cout << "parent: pid_child=" << pid_child <<
                 " ; pid_parent=" << pid_parent << std::endl;
    bool child_death_detected=false;
    int status;
    pid_t p;
    p = waitpid(pid_child, &status, WNOHANG);
    if(p == 0)
    {
      child_death_detected = false;
    }
    else if(p > 0)
    {
      child_death_detected = true;
    }
    else
    {
      ASSERT_TRUE(false && "waitpid failed");
    }
    ASSERT_FALSE(child_death_detected);

    usleep(200000);
    p = waitpid(pid_child, &status, WNOHANG);
//    std::cout << "p=" << p << std::endl;
    if(p == 0)
    {
      child_death_detected = false;
    }
    else if(p > 0)
    {
      child_death_detected = true;
    }
    else
    {
      ASSERT_TRUE(false && "waitpid failed");
    }
    ASSERT_TRUE(child_death_detected);
    wait(nullptr);
  }
  else
  {
    ASSERT_TRUE(false && "fork failed");
  }
}
