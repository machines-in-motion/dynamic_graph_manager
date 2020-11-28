/**
 * @file ros_interpreter.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 *
 * This file declares a ros bridge on top of Python interpretor. It is used
 * an asynchronuous communcation between the user and the controller.
 */

#ifndef DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
#define DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH

#include "dynamic-graph/python/interpreter.hh"
#include "dynamic_graph_manager/ros.hpp"

namespace dynamic_graph_manager
{


/// \brief This class wraps the implementation of the runCommand
/// service.
///
/// This takes as input a ROS node handle and do not handle the
/// callback so that the service behavior can be controlled from
/// the outside.
class RosPythonInterpreterServer
{
public:
    /**
     * @brief run_python_command_callback_t define a std::function to be used
     * as callback to the rclcpp::service.
     *
     * The first argument of "runCommandCallback"
     * (const std::string & command) is bound to
     * (dynamic_graph_manager::srv::RunPythonCommand::Request).
     * And the second argument (std::string &result) is bound to
     * (dynamic_graph_manager::srv::RunPythonCommand::Response)
     */
    typedef std::function<void(RunPythonCommandRequestPtr,
                               RunPythonCommandResponsePtr)>
        run_python_command_callback_t;

    /**
     * @brief run_python_file_callback_t define a std::function to be used as
     * callback to the rclcpp::service.
     *
     * The first argument of "runPythonFileCallback"
     * (std::string ifilename) is bound to
     * (dynamic_graph_manager::srv::RunPythonCommand::Request).
     * And a fake second argument is simulated in
     * (dynamic_graph_manager::srv::RunPythonCommand::Response)
     */
    typedef std::function<void(RunPythonFileRequestPtr,
                               RunPythonFileResponsePtr)>
        run_python_file_callback_t;

    /**
     * @brief RosPythonInterpreterServer is the unique constructor of the class
     * @param node_handle is the RosNode used to advertize the
     *        rclcpp::services
     */
    explicit RosPythonInterpreterServer();

    /**
     * @brief ~RosPythonInterpreterServer is the default destructor of the class
     */
    ~RosPythonInterpreterServer();

    /**
     * @brief run_python_command used the python interpreter to execute the
     * input command
     * @param[in] command is the user input string command.
     * @param[out] result is the numerical result of the operation done.
     * @param[out] out is the string representation of the results.
     * @param[out] err is the output error string from python.
     */
    void run_python_command(const std::string& command,
                            std::string& result,
                            std::string& out,
                            std::string& err);

    /**
     * @brief run_python_file executes the input scripts in the python
     * interpreter
     * @param ifilename is the path to the script to execute
     */
    void run_python_file(const std::string ifilename,
                         std::string& standard_error);

    /**
     * @brief start_ros_service advertize the "run_python_command" and
     * "run_python_scripts" ros services
     */
    void start_ros_service();

protected:
    /**
     * @brief runCommandCallback is the "run_python_command" ros service
     * callback function.
     * @param req is the request. it is defined as a string in the
     *        RunCommand.msg
     * @param res is the request. it is defined as a string in the
     *        RunCommand.msg
     * @return true
     */
    void runCommandCallback(RunPythonCommandRequestPtr req,
                            RunPythonCommandResponsePtr res);

    /**
     * @brief runCommandCallback is the "run_python_script" ros service callback
     *        function.
     * @param req is the request. it is defined as a string in the
     *        RunCommand.msg
     * @param res is the request. it is defined as a string in the
     *        RunCommand.msg
     * @return true
     */
    void runPythonFileCallback(RunPythonFileRequestPtr req,
                               RunPythonFileResponsePtr res);

private:
    /**
     * @brief interpreter_ is the python interpreter itself
     */
    dynamicgraph::python::Interpreter interpreter_;
    /**
     * @brief ros_node_ is reference to the RosNode used to
     * advertize the rclcpp::services
     */
    RosNodePtr ros_node_;
    /**
     * @brief run_python_command_srv_ is the "run_python_command" rclcpp::service
     * c++ object
     *
     * This kind of ros object require *NOT* to be destroyed. otherwize the
     * rclcpp::service is cancelled. This is the reason why this object is an
     * attribute of the class
     */
    RunPythonCommandServerPtr run_python_command_srv_;
    /**
     * @brief run_python_file_srv_ is the "run_python_script" rclcpp::service c++
     * object
     *
     * This kind of ros object require *NOT* to be destroyed. otherwize the
     * rclcpp::service is cancelled. This is the reason why this object is an
     * attribute of the class
     */
    RunPythonFileServerPtr run_python_file_srv_;
};

}  // namespace dynamic_graph_manager

#endif  //! DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
