/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Common include and declaration for the tests.
 */

#pragma once

#include <gtest/gtest.h>
#include <signal.h>
#include <sys/types.h>

#include <chrono>
#include <thread>

// We use another class of this package which has an independant unittest.
#include "dynamic_graph_manager/ros.hpp"

// The class to test
#include "dynamic_graph_manager/dynamic_graph_manager.hpp"

// Service for the user command unittest
#include "mim_msgs/srv/test_user_cmd_bool.hpp"

// SimpleDGM class, the header is located in the demos/ folder of this repos.
#include "simple_dgm.hpp"

using namespace std::chrono_literals;
using namespace dynamic_graph_manager;

/**
 * @brief Global ros node name used unitquely for the unit-tests.
 */
const std::string ROS_NODE_NAME = "test_dgm";

void start_dg_ros_service()
{
    // Service name.
    std::string service_name = "/dynamic_graph_manager/start_dynamic_graph";
    // Create a client from a temporary node.
    RosNodePtr ros_node = get_ros_node(ROS_NODE_NAME);
    auto client = ros_node->create_client<std_srvs::srv::Empty>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));

    // Fill the command message.
    std_srvs::srv::Empty::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Empty::Request>();
    std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> response;
    // Call the service.
    response = client->async_send_request(request);
    // Wait for the answer.
    ASSERT_TRUE(response.valid());
    // wait_for_response(response);
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
}

void stop_dg_ros_service()
{
    // Service name.
    std::string service_name = "/dynamic_graph_manager/stop_dynamic_graph";
    // Create a client from a temporary node.
    RosNodePtr ros_node = get_ros_node(ROS_NODE_NAME);
    auto client = ros_node->create_client<std_srvs::srv::Empty>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));

    // Fill the command message.
    std_srvs::srv::Empty::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Empty::Request>();
    std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> response;
    // Call the service.
    response = client->async_send_request(request);
    // Wait for the answer.
    ASSERT_TRUE(response.valid());
    // wait_for_response(response);
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
}

void start_run_python_command_ros_service(const std::string& cmd,
                                          std::string& result,
                                          std::string& standard_error,
                                          std::string& standard_output)
{
    // Service name.
    std::string service_name = "/dynamic_graph_manager/run_python_command";
    // Create a client from a temporary node.
    RosNodePtr ros_node = get_ros_node(ROS_NODE_NAME);
    auto client =
        ros_node->create_client<mim_msgs::srv::RunPythonCommand>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));

    // Fill the command message.
    mim_msgs::srv::RunPythonCommand::Request::SharedPtr request =
        std::make_shared<mim_msgs::srv::RunPythonCommand::Request>();
    std::shared_future<mim_msgs::srv::RunPythonCommand::Response::SharedPtr>
        response;
    request->input = cmd;
    // Call the service.
    response = client->async_send_request(request);
    // Wait for the answer.
    ASSERT_TRUE(response.valid());
    // wait_for_response(response);
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
    // Get the results.
    result = response.get()->result;
    standard_error = response.get()->standard_error;
    standard_output = response.get()->standard_output;
}

void start_run_python_script_ros_service(const std::string& file_name,
                                         std::string& result,
                                         std::string& standard_error)
{
    // Service name.
    std::string service_name = "/dynamic_graph_manager/run_python_file";
    // Create a client from a temporary node.
    RosNodePtr ros_node = get_ros_node(ROS_NODE_NAME);
    auto client =
        ros_node->create_client<mim_msgs::srv::RunPythonFile>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));

    // Fill the command message.
    mim_msgs::srv::RunPythonFile::Request::SharedPtr request =
        std::make_shared<mim_msgs::srv::RunPythonFile::Request>();
    std::shared_future<mim_msgs::srv::RunPythonFile::Response::SharedPtr>
        response;
    request->input = file_name;
    // Call the service.
    response = client->async_send_request(request);
    // Wait for the answer.
    ASSERT_TRUE(response.valid());
    // wait_for_response(response);
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
    // Get the results.
    result = response.get()->result;
    standard_error = response.get()->standard_error;
}

void user_cmd_ros_service(bool user_input)
{
    // Service name.
    std::string service_name = "/dynamic_graph_manager/set_a_boolean";
    // Create a client from a temporary node.
    RosNodePtr ros_node = get_ros_node(ROS_NODE_NAME);
    auto client =
        ros_node->create_client<mim_msgs::srv::TestUserCmdBool>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));

    // Fill the command message.
    mim_msgs::srv::TestUserCmdBool::Request::SharedPtr request =
        std::make_shared<mim_msgs::srv::TestUserCmdBool::Request>();
    std::shared_future<mim_msgs::srv::TestUserCmdBool::Response::SharedPtr>
        response;
    request->input_boolean = user_input;
    // Call the service.
    response = client->async_send_request(request);
    // Wait for the answer.
    ASSERT_TRUE(response.valid());
    // wait_for_response(response);
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
    // Get the results.
    ASSERT_TRUE(response.get()->sanity_check);
    // wait until the command has been executed.
    real_time_tools::Timer::sleep_sec(0.5);
}

void display_services(
    std::map<std::string, std::vector<std::string> > list_service_and_types)
{
    for (std::map<std::string, std::vector<std::string> >::const_iterator
             const_it = list_service_and_types.begin();
         const_it != list_service_and_types.end();
         ++const_it)
    {
        std::cerr << const_it->first << "\t\t\t";
        for (unsigned i = 0; i < const_it->second.size(); ++i)
        {
            std::cerr << std::right;
            std::cerr << const_it->second[i] << std::endl;
        }
    }
}

void display_services()
{
    std::map<std::string, std::vector<std::string> > list_service_and_types =
        get_ros_node("display_services")->get_service_names_and_types();
    for (std::map<std::string, std::vector<std::string> >::const_iterator
             const_it = list_service_and_types.begin();
         const_it != list_service_and_types.end();
         ++const_it)
    {
        std::cerr << const_it->first << "\t\t\t";
        for (unsigned i = 0; i < const_it->second.size(); ++i)
        {
            std::cerr << std::right;
            std::cerr << const_it->second[i] << std::endl;
        }
    }
}