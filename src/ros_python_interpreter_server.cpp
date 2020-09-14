/**
 * @file ros_interpreter.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include "dynamic_graph_manager/ros_python_interpreter_server.hpp"

#include <functional>
#include <memory>

namespace dynamic_graph_manager
{
static const int queueSize = 5;

RosPythonInterpreterServer::RosPythonInterpreterServer(RosNodePtr ros_node)
    : interpreter_(),
      ros_node_(ros_node),
      run_python_command_srv_(nullptr),
      run_python_file_srv_(nullptr)
{
}

RosPythonInterpreterServer::~RosPythonInterpreterServer()
{
}

void RosPythonInterpreterServer::start_ros_service()
{
    run_python_command_callback_t runCommandCb =
        std::bind(&RosPythonInterpreterServer::runCommandCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2);
    run_python_command_srv_ =
        ros_node_->create_service<RunPythonCommandSrvType>("run_python_command",
                                                           runCommandCb);

    run_python_file_callback_t runPythonFileCb =
        std::bind(&RosPythonInterpreterServer::runPythonFileCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2);
    run_python_file_srv_ = ros_node_->create_service<RunPythonFileSrvType>(
        "run_python_script", runPythonFileCb);
}

void RosPythonInterpreterServer::runCommandCallback(
    RunPythonCommandRequestPtr req, RunPythonCommandResponsePtr res)
{
    run_python_command(
        req->input, res->result, res->standard_output, res->standard_error);
}

void RosPythonInterpreterServer::runPythonFileCallback(
    RunPythonFileRequestPtr req, RunPythonFileResponsePtr res)
{
    run_python_file(req->input, res->standard_error);
    // FIX: It is just an echo, is there a way to have a feedback?
    res->result = "File parsed";
}

void RosPythonInterpreterServer::run_python_command(const std::string& command,
                                                    std::string& result,
                                                    std::string& out,
                                                    std::string& err)
{
    interpreter_.python(command, result, out, err);
}

void RosPythonInterpreterServer::run_python_file(const std::string ifilename,
                                                 std::string& standard_error)
{
    interpreter_.runPythonFile(ifilename, standard_error);
}

}  // namespace dynamic_graph_manager
