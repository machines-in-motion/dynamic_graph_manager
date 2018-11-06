/**
 * \file device.cpp
 * \brief The robot entity
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file implements the input and output of the DynamicGraph
 */

#include <iostream>

#include <yaml-cpp/yaml_eigen.h>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/debug.h>

#include <dynamic_graph_manager/device.hh>

using namespace std;
using namespace dynamic_graph;

/**
 * @brief str2int used in the switch case, it allows to convert in compile time
 * strings to int
 * @param str is the string to convert
 * @param h is the index of the string to start with
 * @return an integer that correspond to the string
 */
constexpr unsigned int str2int(const char* str, int h = 0)
{
  return !str[h] ?
        5381 : (str2int(str, h+1) * 33) ^ static_cast<unsigned int>(str[h]);
}

/**
 * @brief dynamic_graph::Device::CLASS_NAME must be the name as the actual
 * device class.
 */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Device, "Device");

Device::Device(const std::string& input_name):
  // Call the mother class constructor
  Entity(input_name)
{
  addCommand (
        "initialize",
        dynamicgraph::command::makeCommandVoid1(
          *this,
          &Device::initialize_from_file,
          dynamicgraph::command::docCommandVoid1(
            "Initialize the device from a YAML file",
            "string (valid path to the yaml configuration file)")));
}

void Device::initialize_from_file(const std::string& yaml_file)
{
  YAML::Node params = YAML::LoadFile(yaml_file);
  initialize(params["device"]);
}

void Device::initialize(const YAML::Node& params)
{
  params_ = params;
  /**********************************************
   * create maps for signals and vector<double> *
   **********************************************/
  initialize_maps(params_);

  /******************************
   * registering sensor signals *
   ******************************/
  for (DeviceOutSignalMap::iterator sig_it = sensors_out_.begin();
       sig_it != sensors_out_.end(); ++sig_it)
  {
    signalRegistration(*(sig_it->second));
  }

  /*******************************
   * registering control signals *
   *******************************/
  for (DeviceInSignalMap::iterator sig_it = motor_controls_in_.begin();
       sig_it != motor_controls_in_.end(); ++sig_it)
  {
    signalRegistration(*(sig_it->second));
  }

  /***********************************************************
   * Handle commands and signals called in a synchronous way *
   ***********************************************************/
  periodic_call_before_.addSpecificCommands(*this, commandMap, "before.");
  periodic_call_after_.addSpecificCommands(*this, commandMap, "after.");
}

Device::~Device()
{
  // destruct the sensor signals
  for(DeviceOutSignalMap::iterator it = sensors_out_.begin() ;
      it != sensors_out_.end() ; ++it)
  {
    if(it->second != nullptr)
    {
      delete it->second;
      it->second = nullptr;
    }
  }
  // destruct the motor controls signals
  for(DeviceInSignalMap::iterator it = motor_controls_in_.begin() ;
      it != motor_controls_in_.end() ; ++it)
  {
    if(it->second != nullptr)
    {
      delete it->second;
      it->second = nullptr;
    }
  }

  this->entityDeregistration();
}

void Device::initialize_maps(const YAML::Node& sensors_and_controls)
{
  /**************************************************************
   * Parsing the YAML node and fill in the Eigen::VectosXd maps *
   **************************************************************/
  parse_yaml_node(sensors_and_controls, sensors_map_, motor_controls_map_);

  std::string hardware_name ("");

  /*******************************
   * We iterate over the sensors *
   *******************************/
  sensors_out_.clear();
  for (VectorDGMap::const_iterator sensor_it = sensors_map_.begin();
       sensor_it != sensors_map_.end(); ++sensor_it)
  {
    {
      hardware_name = sensor_it->first;
      ostringstream sig_name;
      sig_name << "Device(" << this->name << ")::"
               << "output(vector" << sensor_it->second.size() << "d)::"
               << hardware_name ;
      sensors_out_[hardware_name] = new OutSignal(sig_name.str());
      sensors_out_[hardware_name]->setConstant(sensors_map_[hardware_name]);
    }
  }

  /********************************
   * We iterate over the controls *
   ********************************/
  for (VectorDGMap::const_iterator control_it = motor_controls_map_.begin();
       control_it != motor_controls_map_.end(); ++control_it)
  {
    {
      hardware_name = control_it->first;
      ostringstream sig_name;
      sig_name << "Device(" << this->name << ")::"
               << "input(vector" << control_it->second.size() << "d)::"
               << hardware_name ;
      motor_controls_in_[hardware_name] =
          new InSignal(nullptr, sig_name.str());
      motor_controls_in_[hardware_name]->setConstant(
            motor_controls_map_[hardware_name]);
    }
  }
}

void Device::set_sensors_from_map(const VectorDGMap& sensors)
{
  // check that all map has the same size
  assert(sensors.size() == sensors_map_.size() &&
         sensors_map_.size() == sensors_out_.size() &&
         "Device::set_sensors_from_map: All maps has to be the same size");
  // we do a copy of the map while checking for sanity
  for(VectorDGMap::const_iterator ext_sensor_it = sensors.begin();
      ext_sensor_it != sensors.end(); ++ext_sensor_it)
  {
    assert(sensors_map_.count(ext_sensor_it->first) &&
           "Device::set_sensors_from_map: All field in the input sensors map\
            exists in the internal copy");
    assert(static_cast<unsigned>(ext_sensor_it->second.size()) ==
           sensors_map_[ext_sensor_it->first].size() &&
           "Device::set_sensors_from_map: the vectors have the same size in the\
            maps");
    sensors_map_[ext_sensor_it->first] = ext_sensor_it->second;
    sensors_out_[ext_sensor_it->first]->setConstant(ext_sensor_it->second);
  }
}

void Device::execute_graph()
{
  /*******************************************
   * Get the time of the last sensor reading *
   *******************************************/
  assert(sensors_out_.size() != 0 && "There exist some sensors.");
  int time = sensors_out_.begin()->second->getTime();
  dgDEBUG(25) << "Time : " << time << std::endl;

  /******************************************************************
   * Run Synchronous commands and evaluate signals outside the main *
   * connected component of the graph.                              *
   ******************************************************************/
  try
  {
    periodic_call_before_.run(time+1);
  }
  catch (std::exception& e)
  {
    std::cerr
        << "exception caught while running periodical commands (before): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    std::cerr
        << "exception caught while running periodical commands (before): "
        << str << std::endl;
  }
  catch (...)
  {
    std::cerr
        << "unknown exception caught while"
        << " running periodical commands (before)" << std::endl;
  }

  /***********************************************************************
   * Run the graph by accessing the values of the signals inside the map *
   ***********************************************************************/
  for(DeviceInSignalMap::const_iterator sig_in_it = motor_controls_in_.begin() ;
      sig_in_it != motor_controls_in_.end() ; ++sig_in_it)
  {
    (*(sig_in_it->second))(time+1);
  }

  /******************************************************************
   * Run Synchronous commands and evaluate signals outside the main *
   * connected component of the graph.                              *
   ******************************************************************/
  try
  {
    // TODO: "time" or "time + 1"
    periodic_call_after_.run(time+1);
  }
  catch (std::exception& e)
  {
    std::cerr
        << "exception caught while running periodical commands (after): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    std::cerr
        << "exception caught while running periodical commands (after): "
        << str << std::endl;
  }
  catch (...)
  {
    std::cerr
        << "unknown exception caught while"
        << " running periodical commands (after)" << std::endl;
  }
}

void Device::get_controls_to_map(VectorDGMap& motor_controls)
{
  // check that all map has the same size
  assert(motor_controls.size() == motor_controls_map_.size() &&
         motor_controls_map_.size() == motor_controls_in_.size() &&
         "Device::get_controls_to_map: All maps has to be the same size");
  // we do a copy of the map while checking for sanity
  for(VectorDGMap::iterator ext_control_it = motor_controls.begin();
      ext_control_it != motor_controls.end(); ++ext_control_it)
  {
    assert(motor_controls_map_.count(ext_control_it->first) &&
           "Device::get_controls_to_map: All field in the input sensors map\
            exists in the internal copy");
    assert(static_cast<unsigned>(ext_control_it->second.size()) ==
           motor_controls_map_[ext_control_it->first].size() &&
           "Device::get_controls_to_map: the vectors have the same size in the\
            maps");

   motor_controls_map_[ext_control_it->first] =
        motor_controls_in_[ext_control_it->first]->accessCopy();
   ext_control_it->second = motor_controls_map_[ext_control_it->first];
  }
}
