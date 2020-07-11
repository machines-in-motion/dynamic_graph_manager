/**
 * @file device_simulator.cpp
 * @author Julian Viereck
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <iostream>

#include "yaml_cpp_catkin/yaml_eigen.h"

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/debug.h>
#include <dynamic-graph/factory.h>

#include <dynamic_graph_manager/device_simulator.hpp>

using namespace std;

namespace dynamic_graph
{
/**
 * @brief dynamic_graph::Device::CLASS_NAME must be the name as the actual
 * device class.
 */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DeviceSimulator, "DeviceSimulator");

DeviceSimulator::DeviceSimulator(const std::string& input_name)
    :  // Call the mother class constructor
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
         sensor_it != sensors_map_.end();
         ++sensor_it)
    {
        {
            hardware_name = sensor_it->first;
            ostringstream sig_name;
            sig_name << "Device(" << this->name << ")::"
                     << "input(vector" << sensor_it->second.size() << "d)::"
                     << "simu_in_" << hardware_name;
            simu_sensors_in_[hardware_name] =
                new InSignal(nullptr, sig_name.str());
            simu_sensors_in_[hardware_name]->setConstant(
                sensors_map_[hardware_name]);
            signalRegistration(*(simu_sensors_in_[hardware_name]));
        }
    }

    /********************************
     * We iterate over the controls *
     ********************************/
    simu_motor_controls_out_.clear();
    for (VectorDGMap::const_iterator control_it = motor_controls_map_.begin();
         control_it != motor_controls_map_.end();
         ++control_it)
    {
        {
            hardware_name = control_it->first;
            ostringstream sig_name;
            sig_name << "Device(" << this->name << ")::"
                     << "output(vector" << control_it->second.size() << "d)::"
                     << "simu_out_" << hardware_name;
            simu_motor_controls_out_[hardware_name] =
                new OutSignal(sig_name.str());
            simu_motor_controls_out_[hardware_name]->setConstant(
                motor_controls_map_[hardware_name]);
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

    /***********************************************
     * We copy the input sensor in the output ones *
     ***********************************************/
    for (VectorDGMap::const_iterator sensor_it = sensors_map_.begin();
         sensor_it != sensors_map_.end();
         ++sensor_it)
    {
        sensors_out_[sensor_it->first]->setConstant(
            simu_sensors_in_[sensor_it->first]->accessCopy());
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
         control_it != motor_controls_map_.end();
         ++control_it)
    {
        simu_motor_controls_out_[control_it->first]->setConstant(
            motor_controls_in_[control_it->first]->accessCopy());
    }
}

}  // namespace dynamic_graph
