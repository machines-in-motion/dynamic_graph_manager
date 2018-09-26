#include "single_motor/dgm_single_motor.hh"

namespace dynamic_graph_manager_demo{

DemoSingleMotor::DemoSingleMotor(): dynamic_graph::DynamicGraphManager()
{

}

void DemoSingleMotor::initialize_hardware_communication_process()
{
  if(is_real_robot_)
  {

  }else{

  }
}

void DemoSingleMotor::get_sensors_to_map(
    dynamic_graph::VectorDGMap& map)
{
  if(is_real_robot_)
  {

  }else{

  }
}

void DemoSingleMotor::set_motor_controls_from_map(
    const dynamic_graph::VectorDGMap& map)
{
  if(is_real_robot_)
  {

  }else{

  }
}

} // dynamic_graph_manager_demo
