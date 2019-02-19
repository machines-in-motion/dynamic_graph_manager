/**
 * \file device_simulator.cpp
 * \brief The robot entity
 * \author Julian Viereck
 * \date 2018
 *
 * This file describes a base class to expose a simulator.
 */

#include <iostream>

#include "dynamic_graph_manager/yaml_eigen.h"

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/debug.h>

#include <dynamic_graph_manager/device_simulator.hh>

using namespace std;

namespace dynamic_graph{

/**
 * @brief dynamic_graph::Device::CLASS_NAME must be the name as the actual
 * device class.
 */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DeviceSimulator, "DeviceSimulator");

DeviceSimulator::DeviceSimulator(const std::string& input_name):
  // Call the mother class constructor
  Device(input_name)
{
}

void DeviceSimulator::initialize(const YAML::Node& params)
{
  Device::initialize(params);
  std::string hardware_name = "";
  /*******************************
   * We iterate over the sensors *
   *******************************/
  simu_sensors_in_.clear();
  for (VectorDGMap::const_iterator sensor_it = sensors_map_.begin();
       sensor_it != sensors_map_.end(); ++sensor_it)
  {
    {
      hardware_name = sensor_it->first;
      ostringstream sig_name;
      sig_name << "Device(" << this->name << ")::"
               << "input(vector" << sensor_it->second.size() << "d)::"
               << "simu_in_" 
               << hardware_name ;
      simu_sensors_in_[hardware_name] = new InSignal(nullptr, sig_name.str());
      simu_sensors_in_[hardware_name]->setConstant(sensors_map_[hardware_name]);
      signalRegistration(*(simu_sensors_in_[hardware_name]));
    }
  }

  /********************************
   * We iterate over the controls *
   ********************************/
  simu_motor_controls_out_.clear();
  for (VectorDGMap::const_iterator control_it = motor_controls_map_.begin();
       control_it != motor_controls_map_.end(); ++control_it)
  {
    {
      hardware_name = control_it->first;
      ostringstream sig_name;
      sig_name << "Device(" << this->name << ")::"
               << "output(vector" << control_it->second.size() << "d)::"
               << "simu_out_"
               << hardware_name ;
      simu_motor_controls_out_[hardware_name] = new OutSignal(sig_name.str());
      simu_motor_controls_out_[hardware_name]->setConstant(motor_controls_map_[hardware_name]);
      signalRegistration(*(simu_motor_controls_out_[hardware_name]));
    }
  }
}

void DeviceSimulator::initialize_from_file(const std::string& yaml_file)
{
  YAML::Node params = YAML::LoadFile(yaml_file);
  DeviceSimulator::initialize(params["device"]);
}

void DeviceSimulator::execute_graph()
{
  /*******************************************
   * Get the time of the last sensor reading *
   *******************************************/
  assert(simu_sensors_in_.size() != 0 && "There exist some sensors.");
  // Here the time is the maximum time of all sensors.
  int time = simu_sensors_in_.begin()->second->getTime();
  for(DeviceInSignalMap::const_iterator sig_in_it = simu_sensors_in_.begin() ;
      sig_in_it != simu_sensors_in_.end() ; ++sig_in_it)
  {
    int sig_time = sig_in_it->second->getTime();
    if(sig_time > time)
    {
      time = sig_time;
    }
  }

  /***********************************************
   * We copy the input sensor in the output ones *
   ***********************************************/
  for (VectorDGMap::const_iterator sensor_it = sensors_map_.begin();
       sensor_it != sensors_map_.end(); ++sensor_it)
  {
    simu_sensors_in_[sensor_it->first]->setConstant(
      sensors_out_[sensor_it->first]->access(time));
  }

  /**************************************
   * We call the execution of the graph *
   **************************************/

  // We execute the mother class method
  Device::execute_graph();

  /*************************************************
   * We copy the input control in an output signal *
   *************************************************/
  for (VectorDGMap::const_iterator control_it = motor_controls_map_.begin();
       control_it != motor_controls_map_.end(); ++control_it)
  {
    simu_motor_controls_out_[control_it->first]->setConstant(
      motor_controls_in_[control_it->first]->access(time+1));
  }
}

} // namespace dynamic_graph


