#ifndef DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
#define DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH

#include <ros/ros.h>
#include <dynamic_graph_manager/RunCommand.h>
#include <dynamic_graph_manager/RunPythonFile.h>
#include <dynamic-graph/python/interpreter.hh>

namespace dynamic_graph
{
  /// \brief This class wraps the implementation of the runCommand
  /// service.
  ///
  /// This takes as input a ROS node handle and do not handle the
  /// callback so that the service behavior can be controlled from
  /// the outside.
  class RosPythonInterpreter
  {
  public:

    typedef boost::function<
      bool (dynamic_graph_manager::RunCommand::Request&,
            dynamic_graph_manager::RunCommand::Response&)>
      runCommandCallback_t;

    typedef boost::function<
      bool (dynamic_graph_manager::RunPythonFile::Request&,
            dynamic_graph_manager::RunPythonFile::Response&)>
      runPythonFileCallback_t;

    explicit RosPythonInterpreter (ros::NodeHandle& nodeHandle);
    ~RosPythonInterpreter (){}

    /// \brief Method to start python interpreter and deal with messages.
    /// \param Command string to execute, result, stdout, stderr strings.
    void runCommand(const std::string & command, std::string &result,
            std::string &out, std::string &err);

    /// \brief Method to parse python scripts.
    /// \param Input file name to parse.
    void runPythonFile(std::string ifilename);

    /// Initialize service run_command
    void startRosService ();

  protected:
    /// \brief Run a Python command and return result, stderr and stdout.
    bool runCommandCallback (dynamic_graph_manager::RunCommand::Request& req,
                             dynamic_graph_manager::RunCommand::Response& res);

    /// \brief Run a Python file.
    bool runPythonFileCallback (dynamic_graph_manager::RunPythonFile::Request& req,
                                dynamic_graph_manager::RunPythonFile::Response& res);

  private:
    dynamicgraph::python::Interpreter interpreter_;
    ros::NodeHandle& nodeHandle_;
    ros::ServiceServer runCommandSrv_;
    ros::ServiceServer runPythonFileSrv_;
  };
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
