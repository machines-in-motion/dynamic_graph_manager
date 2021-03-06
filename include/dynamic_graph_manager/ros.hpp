/**
 * @file
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 */

#pragma once

#include <memory>

// ROS includes
#include "mim_msgs/srv/run_python_command.hpp"
#include "mim_msgs/srv/run_python_file.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace dynamic_graph_manager
{
const std::string DG_ROS_NODE_NAME = "dynamic_graph";
const std::string HWC_ROS_NODE_NAME = "hardware_communication";

/*
 * A Bunch of convenient shortcuts.
 */
typedef rclcpp::Node RosNode;
typedef std::shared_ptr<RosNode> RosNodePtr;
typedef rclcpp::executors::MultiThreadedExecutor RosExecutor;
typedef std::shared_ptr<RosExecutor> RosExecutorPtr;

// Python commands shortcuts
typedef mim_msgs::srv::RunPythonCommand RunPythonCommandSrvType;
typedef rclcpp::Service<RunPythonCommandSrvType>::SharedPtr
    RunPythonCommandServerPtr;
typedef rclcpp::Client<RunPythonCommandSrvType>::SharedPtr
    RunPythonCommandClientPtr;
typedef RunPythonCommandSrvType::Request::SharedPtr RunPythonCommandRequestPtr;
typedef std::shared_ptr<RunPythonCommandSrvType::Response>
    RunPythonCommandResponsePtr;
typedef std::shared_future<std::shared_ptr<RunPythonCommandSrvType::Response> >
    RunPythonCommandResponseFuturePtr;

// Parse python file shortcuts.
typedef mim_msgs::srv::RunPythonFile RunPythonFileSrvType;
typedef rclcpp::Service<RunPythonFileSrvType>::SharedPtr RunPythonFileServerPtr;
typedef rclcpp::Client<RunPythonFileSrvType>::SharedPtr RunPythonFileClientPtr;
typedef std::shared_ptr<RunPythonFileSrvType::Request> RunPythonFileRequestPtr;
typedef std::shared_ptr<RunPythonFileSrvType::Response>
    RunPythonFileResponsePtr;
typedef std::shared_future<std::shared_ptr<RunPythonFileSrvType::Response> >
    RunPythonFileResponseFuturePtr;

// Shortcut for chrono constant handling.
typedef std::chrono::duration<int64_t, std::ratio<1, 1> > DurationSec;

// Used to save the hardware service pointers.
typedef std::shared_ptr<rclcpp::ServiceBase> ServiceBasePtr;
typedef rclcpp::Service<std_srvs::srv::Empty>::SharedPtr EmptyServicePtr;
typedef std_srvs::srv::Empty EmptyServiceType;

/**
 * @brief rosInit is a global method that uses the structure GlobalRos.
 * Its role is to create the ros::nodeHandle and the ros::spinner once at the
 * first call. It always returns a reference to the node hanlde.
 * @return the reference of GLOBAL_ROS_VAR.node_.
 */
RosNodePtr get_ros_node(std::string node_name);

/**
 * @brief Add a ros node to the global executor.
 *
 * @param node_name
 */
void ros_add_node_to_executor(const std::string& node_name);

/**
 * @brief Checks if the Node has been created.
 *
 * @param node_name
 * @return true
 * @return false
 */
bool ros_node_exists(std::string node_name);

/**
 * @brief Allow all the created ROS node to perform their callbacks.
 * Warning: blocking function.
 */
void ros_spin();

/**
 * @brief Allow all the created ROS node to perform their callbacks.
 */
void ros_spin_non_blocking();

/**
 * @brief Stop the ros spinning.
 */
void ros_stop_spinning();

/**
 * @brief ros_shutdown shuts down ros and stop the ros spinner with the
 * associate name
 */
void ros_shutdown(std::string node_name);

/**
 * @brief ros_shutdown shuts down ros and stop the ros spinners
 */
void ros_shutdown();

/**
 * @brief Remove all node from the executor and delete it.
 */
void ros_clean();

/**
 * @brief Check if ROS is fine.
 *
 * @param node_name
 * @return true
 * @return false
 */
bool ros_ok();

}  // end of namespace dynamic_graph_manager.
