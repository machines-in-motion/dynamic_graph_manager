/**
 * \file device.cpp
 * \brief The robot entity
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file implements the input and output of the DynamicGraph
 */

#include <Eigen/Geometry>

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

const std::string dynamic_graph::Device::CLASS_NAME = "Device";

Device::Device(const std::string& input_name, YAML::Node params):
  // Call the mother class constructor
  Entity(input_name),
  params_(params)
{
  /**********************************************
   * create maps for signals and vector<double> *
   **********************************************/
  parse_yaml_file(params_);

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
}

void Device::parse_yaml_file(const YAML::Node& sensors_and_controls)
{
  const YAML::Node& sensors = sensors_and_controls["sensors"];
  const YAML::Node& controls = sensors_and_controls["controls"];

  std::string hardware_name ("");
  dg::Vector init_value (1);
  unsigned int size (0);

  /*******************************
   * We iterate over the sensors *
   *******************************/
  for (YAML::const_iterator sensor_it = sensors.begin();
       sensor_it != sensors.end(); ++sensor_it)
  {
    {
      hardware_name = sensor_it->first.as<std::string>();
      size = sensor_it->second["size"].as<unsigned int>();
      init_value.resize(size);
      init_value.setZero();
      ostringstream sig_name;
      sig_name << "Device(" << this->name << ")::"
               << "output(vector" << size << "d)::"
               << hardware_name ;
      sensors_out_[hardware_name] = new DeviceOutSignal(sig_name.str());
      sensors_map_[hardware_name] = std::vector<double>(size, 0.0);
    }
  }

  /********************************
   * We iterate over the controls *
   ********************************/
  for (YAML::const_iterator control_it = controls.begin();
       control_it != controls.end(); ++control_it)
  {
    {
      hardware_name = control_it->first.as<std::string>();
      size = control_it->second["size"].as<unsigned int>();
      init_value.resize(size);
      init_value.setZero();
      ostringstream sig_name;
      sig_name << "Device(" << this->name << ")::"
               << "input(vector" << size << "d)::"
               << hardware_name ;
      motor_controls_in_[hardware_name] =
          new DeviceInSignal(nullptr, sig_name.str());
      motor_controls_map_[hardware_name] = std::vector<double>(size, 0.0);
    }
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
    (*(sig_in_it->second))( time );
  }

  /******************************************************************
   * Run Synchronous commands and evaluate signals outside the main *
   * connected component of the graph.                              *
   ******************************************************************/
  try
  {
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


