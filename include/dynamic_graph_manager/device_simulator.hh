/**
 * \file device_simulator.hh
 * \brief The device used to implement a simulator.
 * \author Julian Viereck
 * \date 2018
 *
 * This file describes a base class to expose a simulator.
 */

#ifndef DEVICE_SIMULATOR_HH
#define DEVICE_SIMULATOR_HH

#include <yaml-cpp/yaml.h>

#include <dynamic-graph/entity.h>
#include <dynamic_graph_manager/device.hh>

namespace dynamic_graph {

  class DeviceSimulator: public dynamic_graph::Device
  {
  public:

    /**
     * @brief DeviceSimulator is the constructor. The name allow the
     * DynamicGraph to identify the entity.
     * @param[in] name is the DynamicGraph identifyer.
     */
    DeviceSimulator(const std::string& input_name);

    /**
     * @brief Destroy the DeviceSimulator object
     */
    ~DeviceSimulator(){}

    /**
     * @brief get_sensor_from_map Returns the current sensor readings.
     * @param sensors
     */
    virtual void get_sensors_to_map(VectorDGMap& sensors);


    /**
     * @brief set_control_to_map Puts the specified controls into the control
     * map.
     * @param controls
     */
    virtual void set_controls_from_map(const VectorDGMap& controls);

    /**
     * @Brief step Reads the current control signal values and use them to
     * simulate one step of the simulator.
     */
    virtual void step() = 0;
  };

} // namespace dynamic_graph


#endif /* #ifndef DEVICE_SIMULATOR_HH */
