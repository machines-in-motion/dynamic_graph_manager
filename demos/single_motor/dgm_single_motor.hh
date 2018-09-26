#ifndef DGM_SINGLE_MOTOR_HH
#define DGM_SINGLE_MOTOR_HH

#include <dynamic_graph_manager/dynamic_graph_manager.hh>

namespace dynamic_graph_manager_demo{

class DemoSingleMotor : public dynamic_graph::DynamicGraphManager
{
public:
  /**
   * @brief DemoSingleMotor is the constructor.
   */
  DemoSingleMotor();

  /**
   * @brief ~DemoSingleMotor is the destructor.
   */
  ~DemoSingleMotor(){}

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
   * @brief ctrl_map
   */
  dynamic_graph::VectorDGMap ctrl_map;
};

} // dynamic_graph_manager_demo

#endif // DGM_SINGLE_MOTOR_HH
