#include <dynamic_graph_manager/ros_interpreter.hh>

namespace dynamic_graph
{
  static const int queueSize = 5;

  RosPythonInterpreter::RosPythonInterpreter (ros::NodeHandle& nodeHandle)
    : interpreter_ (),
      nodeHandle_ (nodeHandle),
      runCommandSrv_ (),
      runPythonFileSrv_ ()
  {
  }

  void RosPythonInterpreter::
  startRosService ()
  {
    runCommandCallback_t runCommandCb =
      boost::bind (&RosPythonInterpreter::runCommandCallback, this, _1, _2);
    runCommandSrv_ =
      nodeHandle_.advertiseService ("run_command", runCommandCb);

    runPythonFileCallback_t runPythonFileCb =
      boost::bind (&RosPythonInterpreter::runPythonFileCallback, this, _1, _2);
    runPythonFileSrv_ =
      nodeHandle_.advertiseService ("run_script", runPythonFileCb);
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
    res.result = "File parsed"; // FIX: It is just an echo, is there a way to have a feedback?
    return true;
  }

  void RosPythonInterpreter::runCommand
  (const std::string & command,
   std::string &result,
   std::string &out,
   std::string &err)
  {
    interpreter_.python(command, result, out, err);
  }

  void RosPythonInterpreter::runPythonFile( std::string ifilename ){
      interpreter_.runPythonFile(ifilename);
  }

} // end of namespace dynamicgraph.

