/**
 * @file tools.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <dynamic_graph_manager/tools.hh>

namespace dynamic_graph
{
void parse_yaml_node(const YAML::Node& sensors_and_controls,
                     VectorDGMap& out_sensors_map,
                     VectorDGMap& out_motor_controls_map)
{
    const YAML::Node& sensors = sensors_and_controls["sensors"];
    const YAML::Node& controls = sensors_and_controls["controls"];
    std::string hardware_name("");
    dynamicgraph::Vector init_value(1);
    int size(0);

    out_sensors_map.clear();
    out_motor_controls_map.clear();

    /*******************************
     * We iterate over the sensors *
     *******************************/
    for (YAML::const_iterator sensor_it = sensors.begin();
         sensor_it != sensors.end();
         ++sensor_it)
    {
        hardware_name = sensor_it->first.as<std::string>();
        size = sensor_it->second["size"].as<int>();
        out_sensors_map[hardware_name] = dynamicgraph::Vector(size);
        out_sensors_map[hardware_name].setZero();
    }

    /********************************
     * We iterate over the controls *
     ********************************/
    for (YAML::const_iterator control_it = controls.begin();
         control_it != controls.end();
         ++control_it)
    {
        hardware_name = control_it->first.as<std::string>();
        size = control_it->second["size"].as<int>();
        out_motor_controls_map[hardware_name] = dynamicgraph::Vector(size);
        out_motor_controls_map[hardware_name].setZero();
    }
}

}  // namespace dynamic_graph