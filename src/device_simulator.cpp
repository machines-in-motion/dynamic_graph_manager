/**
 * \file device_simulator.cpp
 * \brief The robot entity
 * \author Julian Viereck
 * \date 2018
 *
 * This file describes a base class to expose a simulator.
 */

#include <iostream>

#include <yaml-cpp/yaml_eigen.h>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/debug.h>

#include <dynamic_graph_manager/device_simulator.hh>

using namespace std;
using namespace dynamic_graph;

DeviceSimulator::DeviceSimulator(const std::string& input_name):
  // Call the mother class constructor
  Device(input_name)
{
  addCommand (
        "step",
        dynamicgraph::command::makeCommandVoid0(
          *this,
          // In the case the ::step method is virtual, which version of
          // step is invoked here? The one of the base class or the of
          // the *this implementation?
          &DeviceSimulator::step,
          dynamicgraph::command::docCommandVoid0(
            "Perform one step with the simulator.")));
}

void DeviceSimulator::get_sensors_to_map(VectorDGMap& sensors)
{
  // check that all map has the same size
  assert(sensors.size() == sensors_map_.size() &&
         sensors_map_.size() == sensors_out_.size() &&
         "Device::get_sensor_from_map: All maps has to be the same size");
  // we do a copy of the map while checking for sanity
  for(VectorDGMap::iterator ext_sensor_it = sensors.begin();
      ext_sensor_it != sensors.end(); ++ext_sensor_it)
  {
    assert(sensors_map_.count(ext_sensor_it->first) &&
           "Device::set_sensors_from_map: All field in the input sensors map"
           "exists in the internal copy");
    assert(static_cast<unsigned>(ext_sensor_it->second.size()) ==
           sensors_map_[ext_sensor_it->first].size() &&
           "Device::set_sensors_from_map: the vectors have the same size in the"
           "maps");

    sensors_map_[ext_sensor_it->first] =
    		sensors_out_[ext_sensor_it->first]->accessCopy();
    ext_sensor_it->second = sensors_map_[ext_sensor_it->first];
  }
}

void DeviceSimulator::set_controls_from_map(const VectorDGMap& motor_controls)
{
  // check that all map has the same size
  assert(motor_controls.size() == motor_controls_map_.size() &&
         motor_controls_map_.size() == motor_controls_in_.size() &&
         "Device::set_controls_to_map: All maps has to be the same size");
  // we do a copy of the map while checking for sanity
  for(VectorDGMap::const_iterator ext_control_it = motor_controls.begin();
      ext_control_it != motor_controls.end(); ++ext_control_it)
  {
    assert(motor_controls_map_.count(ext_control_it->first) &&
           "Device::get_controls_to_map: All field in the input sensors map\
            exists in the internal copy");
    assert(static_cast<unsigned>(ext_control_it->second.size()) ==
           motor_controls_map_[ext_control_it->first].size() &&
           "Device::get_controls_to_map: the vectors have the same size in the\
            maps");

    motor_controls_map_[ext_control_it->first] = ext_control_it->second;
    motor_controls_in_[ext_control_it->first]->setConstant(ext_control_it->second);
  }
}
