/**
 * @file ros_interpreter.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-05-22
 */

#include <dynamic_graph_manager/ros_interpreter.hh>

namespace dynamic_graph
{
  static const int queueSize = 5;

  RosPythonInterpreter::RosPythonInterpreter (ros::NodeHandle& node_handle)
    : interpreter_ (),
      ros_node_handle_ (node_handle),
      run_python_command_srv_ (),
      run_python_file_srv_ ()
  {
  }

  RosPythonInterpreter::~RosPythonInterpreter()
  {
    run_python_command_srv_.shutdown();
    run_python_file_srv_.shutdown();
  }

  void RosPythonInterpreter::
  start_ros_service ()
  {
    run_python_command_callback_t runCommandCb =
      boost::bind (&RosPythonInterpreter::runCommandCallback, this, _1, _2);
    run_python_command_srv_ =
      ros_node_handle_.advertiseService ("run_python_command", runCommandCb);

    run_python_file_callback_t runPythonFileCb =
      boost::bind (&RosPythonInterpreter::runPythonFileCallback, this, _1, _2);
    run_python_file_srv_ =
      ros_node_handle_.advertiseService ("run_python_script", runPythonFileCb);
  }

  bool
  RosPythonInterpreter::runCommandCallback(
      dynamic_graph_manager::RunCommand::Request& req,
      dynamic_graph_manager::RunCommand::Response& res)
  {
    run_python_command(req.input,
                       res.result,
                       res.standard_output,
                       res.standard_error);
    return true;
  }

  bool
  RosPythonInterpreter::runPythonFileCallback (
      dynamic_graph_manager::RunPythonFile::Request& req,
      dynamic_graph_manager::RunPythonFile::Response& res)
  {
    run_python_file(req.input, res.standard_error);
    // FIX: It is just an echo, is there a way to have a feedback?
    res.result = "File parsed";
    return true;
  }

  void RosPythonInterpreter::run_python_command
  (const std::string & command,
   std::string &result,
   std::string &out,
   std::string &err)
  {
    interpreter_.python(command, result, out, err);
  }

  void RosPythonInterpreter::run_python_file(const std::string ifilename,
                                             std::string& standard_error){
      interpreter_.runPythonFile(ifilename, standard_error);
  }

} // end of namespace dynamicgraph.

