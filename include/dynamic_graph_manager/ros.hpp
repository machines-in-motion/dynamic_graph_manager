/**
 * @file
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 */

#pragma once

#include <memory>

// ROS includes
#include "std_srvs/srv/empty.hpp"
#include "rclcpp/rclcpp.hpp"
#include "dynamic_graph_manager/srv/run_python_command.hpp"
#include "dynamic_graph_manager/srv/run_python_file.hpp"


namespace dynamic_graph_manager
{
typedef rclcpp::Node RosNode;
typedef std::shared_ptr<RosNode> RosNodePtr;
typedef rclcpp::executors::MultiThreadedExecutor RosExecutor;
typedef std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
    RosExecutorPtr;

typedef dynamic_graph_manager::srv::RunPythonCommand RunPythonCommandSrvType;
typedef dynamic_graph_manager::srv::RunPythonFile RunPythonFileSrvType;
typedef rclcpp::Service<RunPythonCommandSrvType>::SharedPtr
    RunPythonCommandServerPtr;
typedef rclcpp::Service<RunPythonFileSrvType>::SharedPtr RunPythonFileServerPtr;
typedef rclcpp::Client<RunPythonCommandSrvType>::SharedPtr
    RunPythonCommandClientPtr;
typedef rclcpp::Client<RunPythonFileSrvType>::SharedPtr RunPythonFileClientPtr;
typedef RunPythonCommandSrvType::Request::SharedPtr
    RunPythonCommandRequestPtr;
typedef std::shared_ptr<RunPythonFileSrvType::Request> RunPythonFileRequestPtr;
typedef std::shared_ptr<RunPythonCommandSrvType::Response>
    RunPythonCommandResponsePtr;
typedef std::shared_ptr<RunPythonFileSrvType::Response>
    RunPythonFileResponsePtr;

typedef std::chrono::duration<int64_t, std::ratio<1, 1> > DurationSec;

typedef rclcpp::Service<std_srvs::srv::Empty>::SharedPtr EmptyServicePtr;
typedef std_srvs::srv::Empty EmptyServiceType;


/**
 * @brief The GlobalRos struct is a structure that allows to gloabally handle
 * the ROS objects
 */
struct GlobalRos
{
    GlobalRos()
    {
        node_ = nullptr;
        executor_ = nullptr;
    }

    /** @brief ~GlobalRos is a specific destructor that stop the ROS activities
     */
    ~GlobalRos()
    {
    }
    /**
     * @brief node is the global node handle used by all ROS objects
     */
    std::shared_ptr<RosNode> node_;
    /**
     * @brief ros_executor_ is a ros object that handles in a global way
     * all the ros callbacks and interruption. Call ros_executor_.start()
     * in order to start handling the callback in a separate thread.
     */
    std::shared_ptr<RosExecutor> executor_;
};

/**
 * @brief rosInit is a global method that uses the structure GlobalRos.
 * Its role is to create the ros::nodeHandle and the ros::spinner once at the
 * first call. It always returns a reference to the node hanlde.
 * @return the reference of GLOBAL_ROS_VAR.node_.
 */
RosNodePtr get_ros_node(std::string node_name);

/**
 * @brief ros_spinner return the executor_. Call dynamic_graph ros_init if
 * ros has not been initialized
 * @return the reference of GLOBAL_ROS_VAR.executor_.
 */
RosExecutorPtr get_ros_executor(std::string node_name);

/**
 * @brief Allow the ROS node to performs it's callbacks. Blocking function.
 */
void ros_spin(std::string node_name);

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
 * @brief Check if a node handle has been created or not.
 */
bool ros_exist(std::string node_name);

/**
 * @brief Check if ROS is fine.
 * 
 * @param node_name 
 * @return true 
 * @return false 
 */
bool ros_ok();

}  // end of namespace dynamic_graph_manager.
