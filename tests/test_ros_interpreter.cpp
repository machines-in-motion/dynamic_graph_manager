/**
 * @file test_ros_interpreter.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <gtest/gtest.h>

#include <memory>
#include <chrono>

#include "dynamic_graph_manager/ros.hpp"
#include "dynamic_graph_manager/ros_python_interpreter_server.hpp"

/***************************
 * SETUP THE TESTING CLASS *
 ***************************/

using namespace std::chrono_literals;
using namespace dynamic_graph_manager;

/**
 * @brief This is the test environment
 */
class TestRosInterpreter : public ::testing::Test
{
protected:
    /**
     * @brief SetUp, is executed before the unit tests
     */
    void SetUp()
    {
        // Create the ros node
        node_name_ = "ros_python_interpreter_rostest";
        get_ros_node(node_name_);
        ros_spin_non_blocking();
    }
    /**
     * @brief TearDown, is executed after the unit tests
     */
    void TearDown()
    {
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
    ASSERT_NO_THROW(RosPythonInterpreterServer rpi;);
}

TEST_F(TestRosInterpreter, test_destructor_no_throw)
{
    ASSERT_NO_THROW({ RosPythonInterpreterServer rpi; });
}

TEST_F(TestRosInterpreter, test_run_cmd_not_available_upon_construction)
{
    /* setup */
    RosPythonInterpreterServer rpi;
    std::map<std::string, std::vector<std::string> > list_service_and_types;
    RosNodePtr ros_node = get_ros_node(node_name_);
    bool simple_service_exists = false;

    // Check if the service exists.
    list_service_and_types = ros_node->get_service_names_and_types();

    for (std::map<std::string, std::vector<std::string> >::const_iterator
             const_it = list_service_and_types.begin();
         const_it != list_service_and_types.end();
         ++const_it)
    {
        std::cerr << const_it->first << "\t";
        for (unsigned int i = 0; i < const_it->second.size(); ++i)
        {
            std::cerr << const_it->second[i] << "\t";
        }
        std::cerr << std::endl;
    }

    /* test */
    // list_service_and_types.("/" + node_name_ + "/run_python_command",
    // false);
    simple_service_exists = true;
    ASSERT_FALSE(simple_service_exists);
}

TEST_F(TestRosInterpreter, test_run_script_not_available_upon_construction)
{
    /* setup */
    RosPythonInterpreterServer rpi;
    /* test */
    ASSERT_FALSE(true);
    // ros::service::exists("/" + node_name_ + "/run_python_command", false));
}

TEST_F(TestRosInterpreter, test_run_cmd_available_after_init)
{
    /* setup */
    RosPythonInterpreterServer rpi;
    rpi.start_ros_service();
    /* test */
    ASSERT_TRUE(false);
    // ros::service::exists(, false));
}

TEST_F(TestRosInterpreter, test_run_script_available_after_init)
{
    /* setup */
    RosPythonInterpreterServer rpi;
    rpi.start_ros_service();
    // run_python_command client.
    RunPythonCommandClientPtr client =
        get_ros_node(node_name_)
            ->create_client<RunPythonCommandSrvType>(
                "/dynamic_graph_manager/run_python_command");

    // test
    ASSERT_TRUE(client->service_is_ready());
}

TEST_F(TestRosInterpreter, test_call_run_command_result)
{
    /* setup */
    // Create the ros python interpreter.
    RosPythonInterpreterServer rpi;
    rpi.start_ros_service();
    // Create clients.
    std::string service_name = "/dynamic_graph_manager/run_python_command";
    RosNodePtr ros_node = get_ros_node(node_name_);
    RunPythonCommandClientPtr client =
        ros_node->create_client<RunPythonCommandSrvType>(service_name);
    // Request.
    RunPythonCommandRequestPtr request;

    ASSERT_TRUE(client->wait_for_service(100ms));
    // Prepare a simple python operation
    request->input = "1 + 1";
    // call the service
    RunPythonCommandResponseFuturePtr response = client->async_send_request(request);
    // Wait for the result.
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);

    /* test */
    ASSERT_EQ(response.get()->result, "2");
}

TEST_F(TestRosInterpreter, test_call_run_command_standard_output)
{
    /* setup */
    // Create the ros python interpreter.
    RosPythonInterpreterServer rpi;
    rpi.start_ros_service();
    // Create clients.
    std::string service_name = "/dynamic_graph_manager/run_python_command";
    RosNodePtr ros_node = get_ros_node(node_name_);
    RunPythonCommandClientPtr client =
        ros_node->create_client<RunPythonCommandSrvType>(service_name);
    // Request.
    RunPythonCommandRequestPtr request;

    ASSERT_TRUE(client->wait_for_service(100ms));
    // Prepare a simple python operation
    request->input = "print(\"Banana\")";
    // call the service
    RunPythonCommandResponseFuturePtr response = client->async_send_request(request);
    // Wait for the result.
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);

    /* test */
    ASSERT_EQ(response.get()->standard_output, "Banana\n");
}

TEST_F(TestRosInterpreter, test_call_run_command_standard_error)
{
    /* setup */
    // Create the ros python interpreter.
    RosPythonInterpreterServer rpi;
    rpi.start_ros_service();
    // Create clients.
    std::string service_name = "/dynamic_graph_manager/run_python_command";
    RosNodePtr ros_node = get_ros_node(node_name_);
    RunPythonCommandClientPtr client =
        ros_node->create_client<RunPythonCommandSrvType>(service_name);
    // Request.
    RunPythonCommandRequestPtr request;

    ASSERT_TRUE(client->wait_for_service(100ms));
    // Prepare a simple python operation
    request->input = "a";
    // call the service
    RunPythonCommandResponseFuturePtr response = client->async_send_request(request);
    // Wait for the result.
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);

    /* test */
    ASSERT_EQ(
        response.get()->standard_error,
        "Traceback (most recent call last):\n  File \"<string>\", line 1, "
        "in <module>\nNameError: name 'a' is not defined\n");
}

TEST_F(TestRosInterpreter, test_call_run_script_result)
{
    /* setup */
    // Create the ros python interpreter.
    RosPythonInterpreterServer rpi;
    rpi.start_ros_service();
    // Create clients.
    std::string service_name = "/dynamic_graph_manager/run_python_file";
    RosNodePtr ros_node = get_ros_node(node_name_);
    RunPythonFileClientPtr client =
        ros_node->create_client<RunPythonFileSrvType>(service_name);
    // Request.
    RunPythonFileRequestPtr request;

    ASSERT_TRUE(client->wait_for_service(100ms));
    // Prepare a simple python operation
    request->input = TEST_CONFIG_PATH + std::string("simple_add.py");
    // call the service
    RunPythonFileResponseFuturePtr response = client->async_send_request(request);
    // Wait for the result.
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);

    /* test */
    ASSERT_EQ(
        response.get()->result, "File parsed");
}

TEST_F(TestRosInterpreter, test_call_run_script_standard_error)
{
    /* setup */
    // Create the ros python interpreter.
    RosPythonInterpreterServer rpi;
    rpi.start_ros_service();
    // Create clients.
    std::string service_name = "/dynamic_graph_manager/run_python_file";
    RosNodePtr ros_node = get_ros_node(node_name_);
    RunPythonFileClientPtr client =
        ros_node->create_client<RunPythonFileSrvType>(service_name);
    // Request.
    RunPythonFileRequestPtr request;

    ASSERT_TRUE(client->wait_for_service(100ms));
    // Prepare a simple python operation
    request->input = TEST_CONFIG_PATH + std::string("simple_add_fail.py");
    // call the service
    RunPythonFileResponseFuturePtr response = client->async_send_request(request);
    // Wait for the result.
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
     // prepare the test
    std::string error_first_part = "a = 1 + 1 + b";
    std::string error_second_part = "NameError: name 'b' is not defined";
    std::size_t found_first_part =
        response.get()->standard_error.find(error_first_part);
    std::size_t found_second_part =
        response.get()->standard_error.find(error_second_part);

    /* test */
    ASSERT_TRUE(found_first_part != std::string::npos);
    ASSERT_TRUE(found_second_part != std::string::npos);
}
