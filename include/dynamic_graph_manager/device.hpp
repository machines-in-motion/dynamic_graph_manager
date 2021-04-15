/**
 * @file device.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef DEVICE_HH
#define DEVICE_HH

#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/linear-algebra.h>

#include <dynamic_graph_manager/periodic-call.hpp>
#include <dynamic_graph_manager/tools.hpp>

#include "yaml_utils/yaml_cpp_fwd.hpp"

namespace dynamic_graph_manager
{
typedef dynamicgraph::Signal<dynamicgraph::Vector, int> OutSignal;
typedef dynamicgraph::SignalPtr<dynamicgraph::Vector, int> InSignal;
typedef std::map<std::string, OutSignal*> DeviceOutSignalMap;
typedef std::map<std::string, InSignal*> DeviceInSignalMap;

class Device : public dynamicgraph::Entity
{
public:
    /**
     * @brief This is the name of the class that is used to store the object
     * in the dynamic graph
     */
    static const std::string CLASS_NAME;

    /**
     * @brief getClassName is an overloaded function from the class Entity.
     * It is used to access the class name and do
     * @return the name of the device class
     */
    virtual const std::string& getClassName(void) const
    {
        return CLASS_NAME;
    }

    /**
     * @brief Device is the constructor. The name allow the DynamicGraph to
     * identify the entity
     * @param params is the yaml file used to initialize the device
     */
    Device(const std::string& input_name);

    /**
     * @brief ~Device is a default destructor that might overloaded
     */
    virtual ~Device();

    /**
     * @brief initialize is the function that initialize the device from the
     * YAML paramters
     * @param params is the yaml file used to initialize the device
     */
    virtual void initialize(const YAML::Node& params);

    /**
     * @brief initialize_from_file is the function that initialize the device
     * from a YAML file. It loads internally the file and then use the paramters
     * to initialize itself using the "initialize" method.
     * @param params is the yaml file used to initialize the device
     */
    virtual void initialize_from_file(const std::string& yaml_file);

    /**
     * @brief parse_yaml_file fill in the internal maps for sensors and
     * controls.
     */
    void initialize_maps(const YAML::Node& sensors_and_controls);

    /**
     * @brief set_sensors_from_map is a parser that feed the map "sensors" with
     * the hardware sensor readings.
     * @param sensors the sensors data.
     */
    virtual void set_sensors_from_map(const VectorDGMap& sensors);

    /**
     * @brief execute_graph is a fonction that execute the graph.
     *
     * In order it does:
     *  - Execute a first set of synchrounous commands.
     *  - Execute the graph.
     *  - Execute a second set of synchronous commands.
     *
     */
    virtual void execute_graph();

    /**
     * @brief get_controls_to_map is a parser that feed the map "controls" with
     * the output of the DynamicGraph.
     * @param controls is the map containing the controls.
     */
    virtual void get_controls_to_map(VectorDGMap& motor_controls);

    /**
     * @brief Get periodic call before.
     *
     * @return PeriodicCall&
     */
    PeriodicCall& get_periodic_call_before()
    {
        return periodic_call_before_;
    }

    /**
     * @brief Get periodic call after.
     *
     * @return PeriodicCall&
     */
    PeriodicCall& get_periodic_call_after()
    {
        return periodic_call_after_;
    }

    /****************************************************************
     * DEVICE OUPUT SIGNALS // INPUT OF THE GRAPH <=> SENSOR VALUES *
     ****************************************************************/

    /**
     * @brief sensors_out_ is a map of device output signals. They represent
     * all the sensors belonging to the robot.
     */
    DeviceOutSignalMap sensors_out_;

    /**
     * @brief sensors_map_ is a map of dynamicgraph::Vector. They represent
     * all the sensors data measured on the robot.
     */
    VectorDGMap sensors_map_;

    /*******************************************************
     * DEVICE INPUT SIGNALS // OUTPUT OF THE CONTROL GRAPH *
     *******************************************************/

    /**
     * @brief motor_control_in_ is the output motor control for each joint.
     * Feeding this signal *IS MANDATORY* otherwise accessing this data will
     * make the process crash.
     */
    DeviceInSignalMap motor_controls_in_;

    /**
     * @brief motor_controls_map_ is a map of dynamicgraph::Vector. They
     * represent all the controls to be sent to the robot.
     */
    VectorDGMap motor_controls_map_;

protected:
    /**
     * @brief periodic_call_before_ handle the *synchronous* command call on the
     * device between getting the sensor data and sending the commands.
     * Typically used when one wants to evaluate a signal that is not plugged.
     */
    PeriodicCall periodic_call_before_;

    /**
     * @brief periodic_call_after_ handle the *synchronous* command call on the
     * device between getting the sensor data and sending the commands.
     * Typically used when one wants to evaluate a signal that is not plugged.
     */
    PeriodicCall periodic_call_after_;

    /**
     * @brief params is a YAML node that allow the creation of a modular device
     */
    YAML::Node params_;
};

}  // namespace dynamic_graph_manager

#endif /* #ifndef DEVICE_HH */
