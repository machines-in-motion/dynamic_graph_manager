/**
 * \file dgm_single_motor.hh
 * \brief The Control Manager for a single motor
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file declares the DGMSingleMotor class.
 * This code manages the different threads and processes in order to use the
 * DynamicGraph.
 * usage: see the main file: demo_single_motor.cpp
 */
#pragma once
#ifndef DGM_SINGLE_MOTOR_HH
#define DGM_SINGLE_MOTOR_HH

#include <dynamic_graph_manager/dynamic_graph_manager.hh>

namespace dynamic_graph_demo{

class DGMSingleMotor : public dynamic_graph::DynamicGraphManager
{
public:
  /**
   * @brief DemoSingleMotor is the constructor.
   */
  DGMSingleMotor();

  /**
   * @brief ~DemoSingleMotor is the destructor.
   */
  ~DGMSingleMotor(){}

  /**
   * @brief initialize_hardware_communication_process is the function that
   * initialize the hardware.
   */
  void initialize_hardware_communication_process();

  /**
   * @brief get_sensors_to_map acquieres the sensors data and feed it to the
   * input/output map
   * @param[in][out] map is the sensors data filled by this function.
   */
  void get_sensors_to_map(dynamic_graph::VectorDGMap& map);

  /**
   * @brief set_motor_controls_from_map reads the input map that contains the
   * controls and send these controls to the hardware.
   * @param map
   */
  void set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map);

  /**
   * @brief initialize_hardware_communication_process_simu is the function that
   * initialize the hardware for the simulation.
   */
  void initialize_hardware_communication_process_simu();

  /**
   * @brief get_sensors_to_map_simu acquieres the sensors data and feed it to
   * the input/output map for the simulation.
   * @param[in][out] map is the sensors data filled by this function.
   */
  void get_sensors_to_map_simu(dynamic_graph::VectorDGMap& map);

  /**
   * @brief set_motor_controls_from_map_simu reads the input map that contains
   * the controls and send these controls to the hardware for the simulation.
   * @param map
   */
  void set_motor_controls_from_map_simu(const dynamic_graph::VectorDGMap& map);

  /**
   * @brief initialize_hardware_communication_process is the function that
   * initialize the hardware for the real robot.
   */
  void initialize_hardware_communication_process_real_robot();

  /**
   * @brief get_sensors_to_map_real_robot acquieres the sensors data and feed
   * it to the input/output map for the real robot.
   * @param[in][out] map is the sensors data filled by this function.
   */
  void get_sensors_to_map_real_robot(dynamic_graph::VectorDGMap& map);

  /**
   * @brief set_motor_controls_from_map_real_robot reads the input map that
   * contains the controls and send these controls to the hardware for the real
   * robot.
   * @param map
   */
  void set_motor_controls_from_map_real_robot(
      const dynamic_graph::VectorDGMap& map);

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
   * @brief dt_ is the control_period_ cast in double and in second
   */
  double dt_;
};

} // dynamic_graph_demo

#endif // DGM_SINGLE_MOTOR_HH
