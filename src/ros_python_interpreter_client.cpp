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
    if (!ros::isInitialized())
    {
        int argc = 1;
        char* arg0 = strdup(ros_node_name_.c_str());
        char* argv[] = {arg0, nullptr};
        ros::init(argc, argv, ros_node_name_,
                  ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);
        free(arg0);
    }
    mode_handle_ = std::make_shared<ros::NodeHandle>();

    // Create a client for the single python command service of the
    // DynamicGraphManager.
    run_command_service_name_ = "/dynamic_graph/run_python_command";
    connect_to_rosservice_run_python_command();

    // Create a client for the python script reading service of the
    // DynamicGraphManager.
    run_script_service_name_ = "/dynamic_graph/run_python_script";
    connect_to_rosservice_run_python_script();
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
        if (!command_client_.isValid())
        {
            ROS_INFO("Connection to remote server lost. Reconnecting...");
            connect_to_rosservice_run_python_command(timeout_connection_s_);
            return return_string;
        }

        run_command_srv_.request.input = code_string;

        ros::service::waitForService(run_command_service_name_);
        if (!command_client_.call(run_command_srv_))
        {
            // We had an issue calling the service.
            ROS_INFO("Error while parsing command.");
            connect_to_rosservice_run_python_command();
        }
        else
        {
            // Get the standard output (print).
            if (run_command_srv_.response.standard_output != "")
            {
                return_string += run_command_srv_.response.standard_output;
            }

            // Get the error.
            if (run_command_srv_.response.standard_error != "")
            {
                return_string += run_command_srv_.response.standard_error;
            }

            // Get the Result and print it is any.
            if (run_command_srv_.response.result != "None")
            {
                return_string += run_command_srv_.response.result;
            }
        }
    }
    catch (...)
    {
        ROS_INFO("Connection to remote server lost. Reconnecting...");
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
        ROS_INFO("Provided file does not exist: %s", filename.c_str());
        return return_string;
    }

    try
    {
        if (!script_client_.isValid())
        {
            ROS_INFO("Connection to remote server lost. Reconnecting...");
            connect_to_rosservice_run_python_script(timeout_connection_s_);
            return return_string;
        }

        run_script_srv_.request.input = filename;

        if (!script_client_.call(run_script_srv_))
        {
            // We had an issue calling the service.
            ROS_INFO("Error while parsing scripts.");
            return return_string;
        }
        else
        {
            // Get the standard output (print).
            if (run_command_srv_.response.standard_output != "")
            {
                return_string += run_command_srv_.response.standard_output;
            }

            // Get the error.
            if (run_command_srv_.response.standard_error != "")
            {
                return_string += run_command_srv_.response.standard_error;
            }

            // Get the Result and print it is any.
            if (run_command_srv_.response.result != "None")
            {
                return_string += run_command_srv_.response.result;
                if (!boost::algorithm::ends_with(return_string, "\n"))
                {
                    return_string += "\n";
                }
            }
        }
        return return_string;
    }
    catch (...)
    {
        ROS_INFO("Connection to remote server lost. Reconnecting...");
        connect_to_rosservice_run_python_script(timeout_connection_s_);
        return return_string;
    }
}

}  // namespace dynamic_graph_manager