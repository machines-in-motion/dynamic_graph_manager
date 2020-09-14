/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Defines a C++ interface for the ros-service of the dynamic-graph
 * python interpreter. (Available in python through bindings.)
 */

#pragma once

#include "dynamic_graph_manager/ros.hpp"

namespace dynamic_graph_manager
{
/**
 * @brief Client of the RosPythonInterpreterServer through rosservices.
 */
class RosPythonInterpreterClient
{
public:
    /**
     * @brief Construct a new RosPythonInterpreterClient object.
     */
    RosPythonInterpreterClient();

    /**
     * @brief Destroy the RosPythonInterpreterClient object.
     */
    ~RosPythonInterpreterClient()
    {
    }

    /**
     * @brief Call the rosservice of the current running DynamicGraphManager.
     *
     * @param code_string To be executed in the embeded python interpreter.
     * @return std::string
     */
    std::string run_python_command(const std::string& code_string);

    /**
     * @brief Call the rosservice of the current running DynamicGraphManager.
     *
     * @param filename
     */
    std::string run_python_script(const std::string& filename);

private:
    /**
     * @brief Connect to a designated service.
     *
     * @tparam RosServiceType
     * @param service_name
     * @param client
     * @param timeout
     */
    template <typename RosServiceType>
    typename rclcpp::Client<RosServiceType>::SharedPtr connect_to_rosservice(
        const std::string& service_name, const DurationSec& timeout)
    {
        typename rclcpp::Client<RosServiceType>::SharedPtr client;
        try
        {
            RCLCPP_INFO(rclcpp::get_logger("RosPythonInterpreterClient"),
                        "Waiting for service %s ...",
                        service_name.c_str());
            // let use wait for the existance of the services
            client =
                ros_node_->create_client<RosServiceType>(service_name.c_str());
            if (client->wait_for_service(timeout))
            {
                RCLCPP_INFO(rclcpp::get_logger("RosPythonInterpreterClient"),
                            "Successfully connected to %s",
                            service_name.c_str());
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("RosPythonInterpreterClient"),
                            "Failed to connect to %s",
                            service_name.c_str());
            }
        }
        catch (...)
        {
            throw std::runtime_error(service_name + " not available.");
        }
        return client;
    }

    /**
     * @brief Connects to the RunCommand rosservice.
     *
     * @param timeout [in]
     */
    void connect_to_rosservice_run_python_command(
        const DurationSec& timeout = DurationSec(-1))
    {
        command_client_ = connect_to_rosservice<RunPythonCommandSrvType>(
            run_command_service_name_, timeout);
    }

    /**
     * @brief Connects to the RunPythonFile rosservice.
     *
     * @param timeout [in]
     */
    void connect_to_rosservice_run_python_script(
        const DurationSec& timeout = DurationSec(-1))
    {
        script_client_ = connect_to_rosservice<RunPythonFileSrvType>(
            run_script_service_name_, timeout);
    }

private:
    /**
     * @brief Name of the local ros_node;
     */
    std::string ros_node_name_;

    /** @brief Handle for manipulating ROS objects. */
    RosNodePtr ros_node_;

    /** @brief  Name of the DynamicGraphManager RosPythonInterpreterServer
     * rosservice for running a python command. */
    std::string run_command_service_name_;

    /** @brief Rosservice to run a python command.
     * @see run_command_service_name_ */
    RunPythonCommandClientPtr command_client_;

    /** @brief Input of the rosservice. */
    RunPythonCommandRequestPtr run_command_request_;

    /** @brief  Name of the DynamicGraphManager RosPythonInterpreterServer
     * rosservice for running a python script. */
    std::string run_script_service_name_;

    /** @brief Rosservice to run a python script.
     * @see run_command_service_name_ */
    RunPythonFileClientPtr script_client_;

    /** @brief Input of the rosservice. */
    RunPythonFileRequestPtr run_file_request_;

    /** @brief timeout used during the connection to the rosservices in
     * seconds. */
    DurationSec timeout_connection_s_;
};

}  // namespace dynamic_graph_manager