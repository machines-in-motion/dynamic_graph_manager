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

#include "dynamic_graph_manager/RunCommand.h"
#include "dynamic_graph_manager/RunPythonFile.h"
#include "dynamic_graph_manager/ros_init.hpp"

namespace dynamic_graph_manager
{
/**
 * @brief Client of the RosPythonInterpreter through rosservices.
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
     * @tparam RosService
     * @param service_name
     * @param client
     * @param timeout
     */
    template <typename RosService>
    ros::ServiceClient connect_to_rosservice(const std::string& service_name,
                                             double timeout)
    {
        ros::ServiceClient client;
        try
        {
            ROS_INFO("Waiting for service %s ...", service_name.c_str());
            // let use wait for the existance of the services
            ros::NodeHandle& n = dynamic_graph::ros_init(ros_node_name_);
            client = n.serviceClient<RosService>(service_name.c_str());
            client.waitForExistence(ros::Duration(timeout));
            if (client.isValid())
            {
                ROS_INFO("Successfully connected to %s", service_name.c_str());
            }
            else
            {
                ROS_INFO("Failed to connect to %s", service_name.c_str());
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
    void connect_to_rosservice_run_python_command(double timeout = -1.0)
    {
        command_client_ =
            connect_to_rosservice<dynamic_graph_manager::RunCommand>(
                run_command_service_name_, timeout);
    }

    /**
     * @brief Connects to the RunPythonFile rosservice.
     *
     * @param timeout [in]
     */
    void connect_to_rosservice_run_python_script(double timeout = -1.0)
    {
        script_client_ =
            connect_to_rosservice<dynamic_graph_manager::RunPythonFile>(
                run_script_service_name_, timeout);
    }

private:
    /**
     * @brief Name of the local ros_node;
     */
    std::string ros_node_name_;

    /** @brief  Name of the DynamicGraphManager RosPythonInterpreter rosservice
     * for running a python command. */
    std::string run_command_service_name_;

    /** @brief Rosservice to run a python command.
     * @see run_command_service_name_ */
    ros::ServiceClient command_client_;

    /** @brief Input/Ouput of the rosservice. */
    dynamic_graph_manager::RunCommand run_command_srv_;

    /** @brief  Name of the DynamicGraphManager RosPythonInterpreter rosservice
     * for running a python script. */
    std::string run_script_service_name_;

    /** @brief Rosservice to run a python script.
     * @see run_command_service_name_ */
    ros::ServiceClient script_client_;

    /** @brief Input/Ouput of the rosservice. */
    dynamic_graph_manager::RunPythonFile run_script_srv_;

    /** @brief timeout used during the connection to the rosservices in
     * seconds. */
    double timeout_connection_s_;
};

}  // namespace dynamic_graph_manager