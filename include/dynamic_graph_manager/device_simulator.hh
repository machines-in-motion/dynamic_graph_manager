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
  // QUESTION@Revier: Is there a way to import these defines from the
  // device.hh and avoid reimporting them here?
  typedef dynamicgraph::Signal<dynamicgraph::Vector,int>  OutSignal;
  typedef dynamicgraph::SignalPtr<dynamicgraph::Vector,int>  InSignal;
  typedef std::map<std::string, OutSignal* > DeviceOutSignalMap;
  typedef std::map<std::string, InSignal* > DeviceInSignalMap;

  class DeviceSimulator: public dynamic_graph::Device
  {
  public:

    /**
     * @brief DeviceSimulator is the constructor. The name allow the DynamicGraph to
     * identify the entity
     * @param params is the yaml file used to initialize the device
     */
    DeviceSimulator(const std::string& input_name);

    /**
     * @brief get_sensor_from_map Returns the current sensor readings.
     * @param sensors
     */
    virtual void get_sensors_to_map(VectorDGMap& sensors);


    /**
     * @brief set_control_to_map Puts the specified controls into the control map.
     */
    virtual void set_controls_from_map(const VectorDGMap& sensors);

    /**
     * @Brief step Reads the current control signal values and use them to
     * simulate one step of the simulator.
     */
    virtual void step() = 0;
  };

} // namespace dynamic_graph


#endif /* #ifndef DEVICE_SIMULATOR_HH */
