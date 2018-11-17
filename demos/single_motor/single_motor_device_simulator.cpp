/**
 * \file single_motor_device_simulator.hh
 * \brief The Control Manager for a single motor
 * \author Julian Viereck
 * \date 2018
 *
 * Implements a single motor simulator.
 */

#include <yaml-cpp/yaml_eigen.h>

#include <dynamic-graph/factory.h>
#include <dynamic_graph_manager/dynamic_graph_manager.hh>
#include <dynamic_graph_manager/device_simulator.hh>

namespace dynamic_graph_demo {

class SingleMotorDeviceSimulator: public dynamic_graph::DeviceSimulator
{
public:
  static const std::string CLASS_NAME;

  /**
   * @brief getClassName is an overloaded function from the class Entity.
   * It is used to access the class name and do
   * @return the name of the device class
   */
  virtual const std::string& getClassName(void) const {
    return CLASS_NAME;
  }

  SingleMotorDeviceSimulator(const std::string& input_name):
      dynamic_graph::DeviceSimulator(input_name)
  {
  };

  /**
   * @brief initialize_from_file Initializes the device from the yaml file.
   * Overloads the default initialize_from_file to parse additional simulation relevant information.
   */
  virtual void initialize_from_file(const std::string& yaml_file)
  {
    YAML::Node params = YAML::LoadFile(yaml_file);
    initialize(params["device"]);

    motor_i_ = params["motor_I"].as<double>();
    motor_KT_ = params["motor_KT"].as<double>();

    dt_ = params["hardware_communication"]["control_period"].as<double>() / pow(10.0,9.0);
    motor_pos_ = 0.;
    motor_vel_ = 0.;
    motor_acc_ = 0.;

    // Initialize the sensor signals.
    sensors_map_["encoder"][0] = motor_pos_;
    set_sensors_from_map(sensors_map_);
  }

  /**
   * @brief step Reads the current control signals, simulates for one dt step and writes the new
   * state into the sensor values.
   */
  virtual void step() {
    get_controls_to_map(motor_controls_map_);

    // Perform the simulation.
    motor_pos_ = motor_pos_ + motor_vel_ * dt_ +
                    motor_acc_ * 0.5 * dt_ * dt_;
    motor_vel_ = motor_vel_ + motor_acc_ * dt_;
    motor_acc_ = motor_controls_map_["torque"][0] / motor_i_;
    sensors_map_["encoder"][0] = motor_pos_;

    set_sensors_from_map(sensors_map_);
  };

private:

  /**
   * @brief motor_acc_ is the simple motor acceleration
   */
  double motor_acc_;

  /**
   * @brief motor_vel_ is the simple motor velocity
   */
  double motor_vel_;

  /**
   * @brief motor_pos_ is the simple motor position
   */
  double motor_pos_;

  /**
   * @brief motor_i_ is the motor inertia
   */
  double motor_i_;

  /**
   * @brief motor_KT_ is the current to torque ration [Nm/A]
   */
  double motor_KT_;

  /**
   * @brief dt_ is the control_period_ cast in double and in second
   */
  double dt_;


};

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SingleMotorDeviceSimulator, "SingleMotorDeviceSimulator");

} // END  dynamic_graph_demo
