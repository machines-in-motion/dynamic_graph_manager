/**
 * \file ros_interpreter.hh
 * \brief the user interface
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file declares a ros bridge on top of Python interpretor. It is used
 * an asynchronuous communcation between the user and the controller.
 */

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
    /**
     * @brief run_python_command_callback_t define a boost::function to be used as
     * callback to the ros::service.
     *
     * The first argument of "runCommandCallback"
     * (const std::string & command) is bound to
     * (dynamic_graph_manager::RunCommand::Request).
     * And the second argument (std::string &result) is bound to
     * (dynamic_graph_manager::RunCommand::Response)
     */
    typedef boost::function<
      bool (dynamic_graph_manager::RunCommand::Request&,
            dynamic_graph_manager::RunCommand::Response&)>
      run_python_command_callback_t;

    /**
     * @brief run_python_file_callback_t define a boost::function to be used as
     * callback to the ros::service.
     *
     * The first argument of "runPythonFileCallback"
     * (std::string ifilename) is bound to
     * (dynamic_graph_manager::RunCommand::Request).
     * And a fake second argument is simulated in
     * (dynamic_graph_manager::RunCommand::Response)
     */
    typedef boost::function<
      bool (dynamic_graph_manager::RunPythonFile::Request&,
            dynamic_graph_manager::RunPythonFile::Response&)>
      run_python_file_callback_t;

    /**
     * @brief RosPythonInterpreter is the unique constructor of the class
     * @param node_handle is the ros::nodeHandle used to advertize the
     *        ros::services
     */
    explicit RosPythonInterpreter (ros::NodeHandle& node_handle);

    /**
      * @brief ~RosPythonInterpreter is the default destructor of the class
      */
    ~RosPythonInterpreter ();

    /**
     * @brief run_python_command used the python interpreter to execute the input command
     * @param[in] command is the user input string command.
     * @param[out] result is the numerical result of the operation done.
     * @param[out] out is the string representation of the results.
     * @param[out] err is the output error string from python.
     */
    void run_python_command(const std::string & command, std::string &result,
            std::string &out, std::string &err);

    /**
     * @brief run_python_file executes the input scripts in the python interpreter
     * @param ifilename is the path to the script to execute
     */
    void run_python_file(const std::string ifilename);

    /**
     * @brief start_ros_service advertize the "run_python_command" and "run_python_scripts"
     *        ros services
     */
    void start_ros_service ();

  protected:
    /**
     * @brief runCommandCallback is the "run_python_command" ros service callback
     *        function.
     * @param req is the request. it is defined as a string in the
     *        RunCommand.msg
     * @param res is the request. it is defined as a string in the
     *        RunCommand.msg
     * @return true
     */
    bool runCommandCallback (dynamic_graph_manager::RunCommand::Request& req,
                             dynamic_graph_manager::RunCommand::Response& res);

    /**
     * @brief runCommandCallback is the "run_python_script" ros service callback
     *        function.
     * @param req is the request. it is defined as a string in the
     *        RunCommand.msg
     * @param res is the request. it is defined as a string in the
     *        RunCommand.msg
     * @return true
     */
    bool runPythonFileCallback (
        dynamic_graph_manager::RunPythonFile::Request& req,
        dynamic_graph_manager::RunPythonFile::Response& res);

  private:
    /**
     * @brief interpreter_ is the python interpreter itself
     */
    dynamicgraph::python::Interpreter interpreter_;
    /**
     * @brief ros_node_handle_ is reference to the ros::NodeHandle used to advertize
     * the ros::services
     */
    ros::NodeHandle& ros_node_handle_;
    /**
     * @brief run_python_command_srv_ is the "run_python_command" ros::service c++ object
     *
     * This kind of ros object require *NOT* to be destroyed. otherwize the
     * ros::service is cancelled. This is the reason why this object is an
     * attribute of the class
     */
    ros::ServiceServer run_python_command_srv_;
    /**
     * @brief run_python_file_srv_ is the "run_python_script" ros::service c++ object
     *
     * This kind of ros object require *NOT* to be destroyed. otherwize the
     * ros::service is cancelled. This is the reason why this object is an
     * attribute of the class
     */
    ros::ServiceServer run_python_file_srv_;
  };
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_BRIDGE_INTERPRETER_HH
