/**
 * \file device.hh
 * \brief The robot entity
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file declares the input and output of the DynamicGraph
 */

#ifndef DEVICE_HH
#define DEVICE_HH

#include <yaml-cpp/yaml.h>

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>

/* short cut of the namespace */
namespace dg = dynamicgraph;

#include "dynamic_graph_manager/periodic-call.hh"
#include <dynamic_graph_manager/matrix-geometry.hh>

namespace dynamic_graph {
  typedef dynamicgraph::Signal<dg::Vector,int>  DeviceOutSignal;
  typedef dynamicgraph::SignalPtr<dg::Vector,int>  DeviceInSignal;
  typedef std::map<std::string, DeviceOutSignal* > DeviceOutSignalMap;
  typedef std::map<std::string, DeviceInSignal* > DeviceInSignalMap;
  typedef std::map<std::string, std::vector<double> > VectorDoubleMap;

  class Device: public dynamicgraph::Entity
  {
  public:

    static const std::string CLASS_NAME;

    /**
     * @brief getClassName is an overloaded fonction from the class Entity.
     * It is used to access the class name and do
     * @return the name of the device class
     */
    virtual const std::string& getClassName(void) const {
      return CLASS_NAME;
    }

    /**
     * @brief Device is the constructor. The name allow the DynamicGraph to
     * identify the entity
     * @param name is the entity name
     */
    Device(const std::string& input_name, YAML::Node params);

    /**
     * @brief ~Device is a default destructor that might overloaded
     */
    virtual ~Device();

    /**
     * @brief parse_yaml_file fill in the internal maps for sensors and controls.
     */
    void parse_yaml_file(const YAML::Node& sensors_and_controls);

    /**
     * @brief get_sensor_from_map
     * @param sensors
     */
    virtual void set_sensors_from_map(
        const std::map<std::string, std::vector<double> >& sensors)=0;

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
     * @brief set_control_to_map is a parser that feed the map "controls" with
     * the output of the DynamicGraph.
     * @param controls is the the map containing the controls.
     */
    virtual void get_controls_to_map(
        std::map<std::string, std::vector<double> >& controls)=0;

    /**
     * @brief display print the name of the device and the current state.
     * @param os is the output stream used to send the message.
     */
    virtual void display ( std::ostream& os ) const
    {
      os << this->name << std::endl;
    }

    /**
     * @brief operator << uses the display method to print the some information
     * about this object
     * @param os is the output stream used to send the message
     * @param r is the device object to print out
     * @return
     */
    friend std::ostream& operator<<(std::ostream& os,const Device& r) {
      r.display(os);
      return os;
    }

    /****************************************************************
     * DEVICE OUPUT SIGNALS // INPUT OF THE GRAPH <=> SENSOR VALUES *
     ****************************************************************/

    /**
     * @brief sensors_in_ is a map of device output signals. They represent
     * all the sensors belonging to the robot.
     */
    DeviceOutSignalMap sensors_out_;

    /**
      * @brief sensors_map_ is a map of vector<double>. They represent
      * all the sensors belonging to the robot.
      */
    VectorDoubleMap sensors_map_;

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
     * @brief motor_controls_map_ is a map of vector<double>. They represent
     * all the motor controls.
     */
    VectorDoubleMap motor_controls_map_;

  private:
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

} // namespace dynamic_graph


#endif /* #ifndef DEVICE_HH */




