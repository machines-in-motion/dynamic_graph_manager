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
      ros_node_handle_.advertiseService ("run_pyhton_script", runPythonFileCb);
  }

  bool
  RosPythonInterpreter::runCommandCallback(
      dynamic_graph_manager::RunCommand::Request& req,
      dynamic_graph_manager::RunCommand::Response& res)
  {
    interpreter_.python(req.input,
                        res.result,
                        res.standardoutput,
                        res.standarderror);
    return true;
  }

  bool
  RosPythonInterpreter::runPythonFileCallback (
      dynamic_graph_manager::RunPythonFile::Request& req,
      dynamic_graph_manager::RunPythonFile::Response& res)
  {
    interpreter_.runPythonFile(req.input);
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

  void RosPythonInterpreter::run_python_file(const std::string ifilename ){
      interpreter_.runPythonFile(ifilename);
  }

} // end of namespace dynamicgraph.

