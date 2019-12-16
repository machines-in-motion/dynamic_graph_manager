/**
 * @file test_ros_interpreter.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-05-22
 */

#include <gtest/gtest.h>
#include <memory>
#include "dynamic_graph_manager/ros_init.hh"
#include "dynamic_graph_manager/ros_interpreter.hh"


/***************************
 * SETUP THE TESTING CLASS *
 ***************************/

using namespace dynamic_graph;

/**
 * @brief This is the test environment
 */
class TestRosInterpreter : public ::testing::Test {
  
protected:
  /**
   * @brief SetUp, is executed before the unit tests
   */
  void SetUp() {
    // Create the ros node
    node_name_ = "ros_python_interpreter_rostest";
    ros_init(node_name_);
  }
  /**
   * @brief TearDown, is executed after the unit tests
   */
  void TearDown() {
    // delete the ros node
    ros_shutdown(node_name_);
  }
  /**
   * @brief Node name
   */
  std::string node_name_;
};

/*****************
 * Start Testing *
 *****************/

TEST_F(TestRosInterpreter, test_constructor_no_throw)
{
  /* setup */
  
  /* test */
  ASSERT_NO_THROW(
    RosPythonInterpreter rpi(ros_init(node_name_));
  );
}

TEST_F(TestRosInterpreter, test_destructor_no_throw)
{
  ASSERT_NO_THROW(
    {
      RosPythonInterpreter rpi(ros_init(node_name_));
    }
  );
}

TEST_F(TestRosInterpreter, test_run_cmd_not_available_upon_construction)
{
  /* setup */
  RosPythonInterpreter rpi(ros_init(node_name_));
  /* test */
  ASSERT_FALSE(ros::service::exists("/" + node_name_ + "/run_python_command",
                                    false));
}

TEST_F(TestRosInterpreter, test_run_script_not_available_upon_construction)
{
  /* setup */
  RosPythonInterpreter rpi(ros_init(node_name_));
  /* test */
  ASSERT_FALSE(ros::service::exists("/" + node_name_ + "/run_python_command",
                                    false));
}

TEST_F(TestRosInterpreter, test_run_cmd_available_after_init)
{
  /* setup */
  RosPythonInterpreter rpi(ros_init(node_name_));
  rpi.start_ros_service();
  /* test */
  ASSERT_TRUE(ros::service::exists("/" + node_name_ + "/run_python_command",
                                    false));
}

TEST_F(TestRosInterpreter, test_run_script_available_after_init)
{
  /* setup */
  RosPythonInterpreter rpi(ros_init(node_name_));
  rpi.start_ros_service();
  /* test */
  ASSERT_TRUE(ros::service::exists("/" + node_name_ + "/run_python_command",
                                    false));
}

TEST_F(TestRosInterpreter, test_call_run_command_result)
{
  /* setup */
  // create the ros interpreter
  RosPythonInterpreter rpi(ros_init(node_name_));
  rpi.start_ros_service();
  // service name is
  std::string service_name = "/" + node_name_ + "/run_python_command";
  // let use wait for the existance of the services
  ros::service::waitForService(service_name, ros::Duration(0.5));
  // fetch the node handle reference
  ros::NodeHandle& node_handle = ros_init(node_name_);
  // Prepare a simple python operation
  dynamic_graph_manager::RunCommand run_com_msg;
  run_com_msg.request.input = "1 + 1";
  // call the service
  ros::service::call(service_name, run_com_msg);

  /* test */
  ASSERT_EQ(run_com_msg.response.result, "2");
}

TEST_F(TestRosInterpreter, test_call_run_command_standard_output)
{
  /* setup */
  // create the ros interpreter
  RosPythonInterpreter rpi(ros_init(node_name_));
  rpi.start_ros_service();
  // service name is
  std::string service_name = "/" + node_name_ + "/run_python_command";
  // let use wait for the existance of the services
  ros::service::waitForService(service_name, ros::Duration(0.5));
  // fetch the node handle reference
  ros::NodeHandle& node_handle = ros_init(node_name_);
  // Prepare a simple python operation
  dynamic_graph_manager::RunCommand run_com_msg;
  run_com_msg.request.input = "print(\"Banana\")";
  // call the service
  ros::service::call(service_name, run_com_msg);

  /* test */
  std::cout << "python stdout: "
            << run_com_msg.response.standard_output 
            << std::endl;
  ASSERT_EQ(run_com_msg.response.standard_output, "Banana\n");
}

TEST_F(TestRosInterpreter, test_call_run_command_standard_error)
{
  /* setup */
  // create the ros interpreter
  RosPythonInterpreter rpi(ros_init(node_name_));
  rpi.start_ros_service();
  // service name is
  std::string service_name = "/" + node_name_ + "/run_python_command";
  // let use wait for the existance of the services
  ros::service::waitForService(service_name, ros::Duration(0.5));
  // fetch the node handle reference
  ros::NodeHandle& node_handle = ros_init(node_name_);
  // Prepare a simple python operation
  dynamic_graph_manager::RunCommand run_com_msg;
  run_com_msg.request.input = "a";
  // call the service
  ros::service::call(service_name, run_com_msg);

  /* test */
  ASSERT_EQ(run_com_msg.response.standard_error,
            "Traceback (most recent call last):\n  File \"<string>\", line 1, "
            "in <module>\nNameError: name 'a' is not defined\n");
}

TEST_F(TestRosInterpreter, test_call_run_script_result)
{
  /* setup */
  // create the ros interpreter
  RosPythonInterpreter rpi(ros_init(node_name_));
  rpi.start_ros_service();
  // service name is
  std::string service_name = "/" + node_name_ + "/run_python_script";
  // let use wait for the existance of the services
  ros::service::waitForService(service_name, ros::Duration(0.5));
  // fetch the node handle reference
  ros::NodeHandle& node_handle = ros_init(node_name_);
  // Prepare a simple python operation
  dynamic_graph_manager::RunPythonFile run_file_msg;
  run_file_msg.request.input = TEST_CONFIG_PATH + std::string("simple_add.py");
  // call the service
  ros::service::call(service_name, run_file_msg);

  /* test */
  ASSERT_EQ(run_file_msg.response.result, "File parsed");
}

TEST_F(TestRosInterpreter, test_call_run_script_standard_error)
{
  /* setup */
  // create the ros interpreter
  RosPythonInterpreter rpi(ros_init(node_name_));
  rpi.start_ros_service();
  // service name is
  std::string service_name = "/" + node_name_ + "/run_python_script";
  // let use wait for the existance of the services
  ros::service::waitForService(service_name, ros::Duration(0.5));
  // fetch the node handle reference
  ros::NodeHandle& node_handle = ros_init(node_name_);
  // Prepare a simple python operation
  dynamic_graph_manager::RunPythonFile run_file_msg;
  run_file_msg.request.input = TEST_CONFIG_PATH + std::string("simple_add_fail.py");
  // call the service
  ros::service::call(service_name, run_file_msg);

  // prepare the test
  std::string error_first_part = "a = 1 + 1 + b" ;
  std::string error_second_part = "NameError: name 'b' is not defined" ;
  std::size_t found_first_part = run_file_msg.response.standard_error.find(
      error_first_part);
  std::size_t found_second_part = run_file_msg.response.standard_error.find(
      error_second_part);;

  /* test */
  ASSERT_TRUE(found_first_part != std::string::npos);
  ASSERT_TRUE(found_second_part != std::string::npos);
}
