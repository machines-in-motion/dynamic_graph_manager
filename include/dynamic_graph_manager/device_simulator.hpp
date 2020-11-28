/**
 * @file device_simulator.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef DEVICE_SIMULATOR_HH
#define DEVICE_SIMULATOR_HH

#include "yaml_utils/yaml_cpp_fwd.hpp"

#include <dynamic-graph/entity.h>
#include <dynamic_graph_manager/device.hpp>

namespace dynamic_graph_manager
{
class DeviceSimulator : public dynamic_graph_manager::Device
{
public:
    /**
     * @brief This is the name of the classe that is used to store the object
     * in the dynamic graph
     */
    static const std::string CLASS_NAME;

    /**
     * @brief DeviceSimulator is the constructor. The name allow the
     * DynamicGraph to identify the entity.
     * @param[in] name is the DynamicGraph identifyer.
     */
    DeviceSimulator(const std::string& input_name);

    /**
     * @brief Destroy the DeviceSimulator object
     */
    ~DeviceSimulator()
    {
    }

    /**
     * @brief This method is hinerited from the Device class. "initialize" is
     * the function that initialize the device from the YAML paramters.
     * @param params is the yaml file used to initialize the device
     */
    virtual void initialize(const YAML::Node& params);

    /**
     * @brief This method is hinerited from the Device class.
     * "initialize_from_file" is the function that initialize the device
     * from a YAML file. It loads internally the file and then use the paramters
     * to initialize itself using the "initialize" method.
     * @param params is the yaml file used to initialize the device
     */
    virtual void initialize_from_file(const std::string& yaml_file);

    /**
     * @brief This method is hinerited from the Device class.
     * "execute_graph" is the method that execute the graph.
     *
     * In order it does:
     *  - <b> Copy the simulation input sensor into the output sensor
     *    signals</b>
     *  - Execute a first set of synchrounous commands.
     *  - Execute the graph.
     *  - Execute a second set of synchronous commands.
     *  - <b> Copy the control input into the simulation control output
     *    signals</b>
     *
     */
    virtual void execute_graph();

    /**
     * @brief simu_motor_controls_out_ is the output motor control for each
     * joint. They are the simulated signals to be plugged to a simulator
     * wrapped as an entity.
     */
    DeviceOutSignalMap simu_motor_controls_out_;

    /**
     * @brief simu_sensors_in_ is a map of device output signals. They represent
     * all the sensors belonging to the robot. Feeding these signals
     * <b> IS MANDATORY </b> otherwise accessing this data will make the
     * process crash.
     */
    DeviceInSignalMap simu_sensors_in_;
};

}  // namespace dynamic_graph_manager

#endif /* #ifndef DEVICE_SIMULATOR_HH */
