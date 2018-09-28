#include "single_motor/dgm_single_motor.hh"

namespace dynamic_graph_demo{

DGMSingleMotor::DGMSingleMotor(): dynamic_graph::DynamicGraphManager()
{
  is_real_robot_ = true;
}

void DGMSingleMotor::initialize_hardware_communication_process()
{
  if(is_real_robot_)
  {
    initialize_hardware_communication_process_real_robot();
  }else{
    initialize_hardware_communication_process_simu();
  }
}

void DGMSingleMotor::get_sensors_to_map(
    dynamic_graph::VectorDGMap& map)
{
  if(is_real_robot_)
  {
    get_sensors_to_map_real_robot(map);
  }else{
    get_sensors_to_map_simu(map);
  }
}

void DGMSingleMotor::set_motor_controls_from_map(
    const dynamic_graph::VectorDGMap& map)
{
  if(is_real_robot_)
  {
    set_motor_controls_from_map_real_robot(map);
  }else{
    set_motor_controls_from_map_simu(map);
  }
}

void DGMSingleMotor::initialize_hardware_communication_process_simu(){
  dt_ = static_cast<double>(control_period_.count()) * pow(10.0,9.0);
  motor_i_ = params_["motor_I"].as<double>();
}

void DGMSingleMotor::get_sensors_to_map_simu(dynamic_graph::VectorDGMap& map)
{
  // here we do very basic integrator
  motor_pos_ = motor_pos_ + motor_vel_ * dt_ +
               motor_acc_ * 0.5 * dt_ * dt_ / motor_i_;
  motor_vel_ = motor_vel_ + motor_acc_ * dt_ / motor_i_;
  motor_acc_ = motor_controls_map_["torque"][0] / motor_i_ ;
  map["encoder"][0] = motor_pos_;
}

void DGMSingleMotor::set_motor_controls_from_map_simu(
    const dynamic_graph::VectorDGMap&)
{
  // do nothing as the integration is in the get_sensors_to_map_simu method
}

void DGMSingleMotor::initialize_hardware_communication_process_real_robot(){

}

void DGMSingleMotor::get_sensors_to_map_real_robot(
    dynamic_graph::VectorDGMap& map)
{

}

void DGMSingleMotor::set_motor_controls_from_map_real_robot(
    const dynamic_graph::VectorDGMap& map)
{

}

} // dynamic_graph_manager_demo
