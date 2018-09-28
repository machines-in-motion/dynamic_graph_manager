/*
 * Demo of the dynamic graph manager on a simple single robot demo.
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
  ros::waitForShutdown();
}

