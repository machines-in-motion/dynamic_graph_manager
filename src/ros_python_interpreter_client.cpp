/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Defines a C++ interface for the ros-service of the dynamic-graph
 * python interpreter. (Available in python through bindings.)
 */

#include "dynamic_graph_manager/ros_python_interpreter_client.hpp"

#include <boost/algorithm/string.hpp>
#include <fstream>

namespace dynamic_graph_manager
{
RosPythonInterpreterClient::RosPythonInterpreterClient()
{
    ros_node_name_ = "ros_python_interpreter_client";

    /** call ros::init */
    if (!rclcpp::is_initialized())
    {
        /** call rclcpp::init */
        int argc = 1;
        char* arg0 = strdup(ros_node_name_.c_str());
        char* argv[] = {arg0, nullptr};
        rclcpp::init(argc, argv);
        free(arg0);
    }
    ros_node_ = rclcpp::Node::make_shared(ros_node_name_);

    // Create a client for the single python command service of the
    // DynamicGraphManager.
    run_command_service_name_ = "/dynamic_graph/run_python_command";
    run_command_request_ = std::make_shared<RunPythonCommandSrvType::Request>();
    connect_to_rosservice_run_python_command();

    // Create a client for the python script reading service of the
    // DynamicGraphManager.
    run_script_service_name_ = "/dynamic_graph/run_python_script";
    run_file_request_ = std::make_shared<RunPythonFileSrvType::Request>();
    connect_to_rosservice_run_python_script();
    timeout_connection_s_ = DurationSec(1);
}

std::string RosPythonInterpreterClient::run_python_command(
    const std::string& code_string)
{
    std::string return_string = "";

    // If the code is empty or just a line break, return an empty string.
    if (code_string == "" || code_string == "\n")
    {
        return return_string;
    }

    try
    {
        if (!command_client_->wait_for_service(timeout_connection_s_))
        {
            RCLCPP_INFO(rclcpp::get_logger("RosPythonInterpreterClient"),
                        "Connection to remote server lost. Reconnecting...");
            connect_to_rosservice_run_python_command(timeout_connection_s_);
            return return_string;
        }

        run_command_request_->input = code_string;

        auto response =
            command_client_->async_send_request(run_command_request_);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(ros_node_, response) ==
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            // Get the standard output (print).
            if (response.get()->standard_output != "")
            {
                return_string += response.get()->standard_output;
            }

            // Get the error.
            if (response.get()->standard_error != "")
            {
                return_string += response.get()->standard_error;
            }

            // Get the Result and print it is any.
            if (response.get()->result != "None")
            {
                return_string += response.get()->result;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Error while parsing command.");
        }
    }
    catch (...)
    {
        RCLCPP_INFO(rclcpp::get_logger("RosPythonInterpreterClient"),
                    "Connection to remote server lost. Reconnecting...");
        connect_to_rosservice_run_python_command(timeout_connection_s_);
    }
    return return_string;
}

std::string RosPythonInterpreterClient::run_python_script(
    const std::string& filename)
{
    std::string return_string = "";
    
    std::ifstream file_if(filename.c_str());
    if (!file_if.good())
    {
        RCLCPP_INFO(rclcpp::get_logger("RosPythonInterpreterClient"),
                    "Provided file does not exist: %s",
                    filename.c_str());
        return return_string;
    }

    try
    {
        if (!script_client_->wait_for_service(timeout_connection_s_))
        {
            RCLCPP_INFO(rclcpp::get_logger("RosPythonInterpreterClient"),
                        "Connection to remote server lost. Reconnecting...");
            connect_to_rosservice_run_python_script(timeout_connection_s_);
            return return_string;
        }

        run_file_request_->input = filename;

        auto response =
            script_client_->async_send_request(run_file_request_);

        if (rclcpp::spin_until_future_complete(ros_node_, response) ==
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            // Get the error.
            if (response.get()->standard_error != "")
            {
                return_string += response.get()->standard_error;
            }

            // Get the Result and print it is any.
            if (response.get()->result != "None")
            {
                return_string += response.get()->result;
                if (!boost::algorithm::ends_with(return_string, "\n"))
                {
                    return_string += "\n";
                }
            }
        }
        else
        {
            // We had an issue calling the service.
            RCLCPP_INFO(rclcpp::get_logger("RosPythonInterpreterClient"),
                        "Error while parsing scripts.");
        }
    }
    catch (...)
    {
        RCLCPP_INFO(rclcpp::get_logger("RosPythonInterpreterClient"),
                    "Connection to remote server lost. Reconnecting...");
        connect_to_rosservice_run_python_script(timeout_connection_s_);
    }
    return return_string;
}

}  // namespace dynamic_graph_manager