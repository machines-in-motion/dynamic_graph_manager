/*
 * Demo of the dynamic graph manager on a simple single robot demo.
 *
 * usage:
 *  1/ Spawn 3 terminal with the catkin workspace are sourced.
 *  2/ In one of them launch this executable located in:
 *    ".../workspace/devel/lib/dynamic_graph_manager/demo_single_motor_simu"
 *    or
 *    ".../workspace/devel/lib/dynamic_graph_manager/demo_single_motor"
 *    The first one launches a very simple simulation of a single motor.
 *    The second one connects to the BLMC api to control a single motor with an
 *    encoder.
 *  3/ In another terminal spawn the python interpreter which the dynamic graph
 *      client:
 *          rosrun dynamic_graph_manager run_command
 *      this will prompt something like:
 *      """
 *      [INFO] [WallTime: 1538638482.420290] waiting for service/dynamic_graph/run_python_command...
 *      [INFO] [WallTime: 1538638482.424085] waiting for service/dynamic_graph/run_python_script...
 *      Interacting with remote server.
 *      >>>
 *      """
 *      this *IS* a python interpreter and you can access the dynamic graph via
 *      the preallocated "robot" object. For example:
 *          >>> robot.startTracer()
 *          >>> robot.stopTracer()
 *          >>> for s in robot.device.signals():
 *          >>>     print s.name
 *  4/ In order to start the dynamic graph one will use a rosservice. So in the
 *      third terminal launch:
 *          rosservice call /dynamic_graph/start_dynamic_graph
 *  5/ And to stop the dynamic graph:
 *          rosservice call /dynamic_graph/stop_dynamic_graph
 *
 * A bank of elementary entities will be provided
 */

#include <single_motor/dgm_single_motor.hh>

int main(int , char* []) {
  std::cout << "Loading paramters from "
            << YAML_PARAMS
            << std::endl;
  YAML::Node param = YAML::LoadFile(YAML_PARAMS);
  dynamic_graph_demo::DGMSingleMotor dgm;

  dgm.initialize(param);
  dgm.run();
  std::cout << "Wait for shutdown, press CTRL+C to close." << std::endl;
  ros::waitForShutdown();
}

