/**
 * \file test_ros_interpreter.cpp
 * \brief DynamicGraphManager unit tests
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file implements a suit of unit tests for the DynamicGraphManager class.
 * @see https://git-amd.tuebingen.mpg.de/amd-clmc/ci_example/wikis/catkin:-how-to-implement-unit-tests
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
  // Setup
  
  // Test
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
  // setup
  RosPythonInterpreter rpi(ros_init(node_name_));
  // test
  ASSERT_FALSE(ros::service::exists("/" + node_name_ + "/run_python_command",
                                    false));
}

TEST_F(TestRosInterpreter, test_run_script_not_available_upon_construction)
{
  // setup
  RosPythonInterpreter rpi(ros_init(node_name_));
  // test
  ASSERT_FALSE(ros::service::exists("/" + node_name_ + "/run_python_command",
                                    false));
}

TEST_F(TestRosInterpreter, test_run_cmd_available_after_init)
{
  // setup
  RosPythonInterpreter rpi(ros_init(node_name_));
  rpi.start_ros_service();
  // test
  ASSERT_TRUE(ros::service::exists("/" + node_name_ + "/run_python_command",
                                    false));
}

TEST_F(TestRosInterpreter, test_run_script_available_after_init)
{
  // setup
  RosPythonInterpreter rpi(ros_init(node_name_));
  rpi.start_ros_service();
  // test
  ASSERT_TRUE(ros::service::exists("/" + node_name_ + "/run_python_command",
                                    false));
}


// TEST_F(TestRosInterpreter, test_call_run_command)
//   // check the run_python_command service
//   dynamic_graph_manager::RunCommand run_com_msg;
//   run_com_msg.request.input = "1 + 1";
//   ros::ServiceClient run_python_command_client =
//       node_handle.serviceClient<dynamic_graph_manager::RunCommand>(
//         "/dynamic_graph/run_python_command");
//   ASSERT_TRUE(run_python_command_client.waitForExistence(ros::Duration(0.5)));
//   ASSERT_TRUE(run_python_command_client.call(run_com_msg));
//   ASSERT_EQ(run_com_msg.response.result, "2");
//   std::cout << ("The run_python_command service has"
//                 " been called successfully") << std::endl;

//   // check the run_script service and the python interpreter
//   dynamic_graph_manager::RunPythonFile run_file_msg;
//   run_file_msg.request.input = TEST_CONFIG_PATH + std::string("simple_add.py");
//   ros::ServiceClient run_script_client =
//       node_handle.serviceClient<dynamic_graph_manager::RunPythonFile>(
//         "/dynamic_graph/run_python_script");
//   ASSERT_TRUE(run_script_client.waitForExistence(ros::Duration(0.5)));
//   ASSERT_TRUE(run_script_client.call(run_file_msg));
//   ASSERT_EQ(run_file_msg.response.result, "File parsed");
//   std::cout << ("The run_script service has"
//                 " been called successfully") << std::endl;

//   // check that "a" exists and has a value "3"
//   run_com_msg.request.input = "a";
//   ASSERT_TRUE(run_python_command_client.call(run_com_msg));
//   ASSERT_EQ(run_com_msg.response.result, "3");
// }