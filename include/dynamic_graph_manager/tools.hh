/**
 * \file tools.hh
 * \brief tools to simplify the code of the DynamicGraphManager
 * \author Maximilien Naveau
 * \date 2018
 *
 * Implementation of the DynamicGraphManager class
 *
 */

#include <dynamic-graph/linear-algebra.h>

#include <yaml-cpp/yaml_eigen.h>


#ifndef TOOLS_HH
#define TOOLS_HH

namespace dynamic_graph {
  /**
   * @brief VectorDGMap is a shortcut for the very long type of the map
   */
  typedef std::map<std::string, dynamicgraph::Vector > VectorDGMap;

  /**
   * @brief parse_yaml_node allows to parse the yaml node to create the
   * according maps of dynamicgraph::Vector.
   * @param[in] sensors_and_controls is the node that contains the hardware info.
   * @param[out] out_sensors_map is the map that contains the sensors data.
   * @param[out] out_motor_controls_map is the map that contains the motor
   * controls data.
   */
  void parse_yaml_node(const YAML::Node& sensors_and_controls,
                       VectorDGMap& out_sensors_map,
                       VectorDGMap& out_motor_controls_map);

} // namespace dynamic_graph

 
#endif /* #ifndef TOOLS_HH */
