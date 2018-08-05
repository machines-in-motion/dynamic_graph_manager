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

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>

/* short cut of the namespace */
namespace dg = dynamicgraph;

#include "dynamic_graph_manager/periodic-call.hh"
#include <dynamic_graph_manager/matrix-geometry.hh>

namespace dynamic_graph {

  class Device: public dynamicgraph::Entity
  {
  public:

    static const std::string CLASS_NAME;

    /**
     * @brief getClassName why create an accessor of a public variable?
     * @return the name of the device class
     */
    virtual const std::string& getClassName(void) const {
      return CLASS_NAME;
    }

  public:

    /**
     * @brief Device is the constructor. The name allow the DynamicGraph to
     * identify the entity
     * @param name is the entity name
     */
    Device(const std::string& name);
    /**
     * @brief ~Device is a default destructor that might overloaded
     */
    virtual ~Device();

    /**
     * @brief increment
     * @param dt
     */
    virtual void increment(const double & dt = 5e-2);

    /**
     * @brief display
     * @param os
     */
    virtual void display(std::ostream& os) const;

    /**
     * @brief operator <<
     * @param os
     * @param r
     * @return
     */
    friend std::ostream& operator<<(std::ostream& os,const Device& r) {
      r.display(os);
      return os;
    }

    /*********************************
     * INPUT SIGNALS / SENSOR VALUES *
     *********************************/

    /**
     * @brief accelerometers_in_ is the list of signals that feed the dynamic
     * graph with the robot different accelerometers data.
     */
    std::vector<dynamicgraph::SignalPtr<dg::Vector,int> > accelerometers_in_;

    /**
     * @brief gyroscopes_in_ is the list of signals that feed the dynamic
     * graph with the robot different gyroscopes data.
     */
    std::vector<dynamicgraph::SignalPtr<dg::Vector,int> > gyroscopes_in_;

    /**
     * @brief wrenchs_in_ is the list of signals that feed the dynamic
     * graph with the robot different 6d force sensors data.
     */
    std::vector<dynamicgraph::SignalPtr<dg::Vector,int> > wrenchs_in_;

    /**
     * @brief joint_pos_in_ feeds the dynamic graph with the joint positions.
     */
    dynamicgraph::SignalPtr<dg::Vector,int> joint_pos_in_;

    /**
     * @brief joint_vel_in_ feeds the dynamic graph with the joint velocities.
     */
    std::vector<dynamicgraph::SignalPtr<dg::Vector,int> > joint_vel_in_;

    /**
     * @brief joint_torques_in_ feeds the dynamic graph with the joint
     * torques.
     */
    std::vector<dynamicgraph::SignalPtr<dg::Vector,int> > joint_torques_in_;

    /**
     * @brief frames_pos_in_ is a list of frame pos as input. The composition
     * of this vector is [x, y, z, q0, q1, q2, q3], i.e. a 3D pose vector
     * and a quaternion
     */
    std::vector<dynamicgraph::SignalPtr<dg::Vector,int> > frames_pos_in_;

    /**********************************
     * OUTPUT SIGNALS / DESIRED VALUE *
     **********************************/

    /**
     * @brief motor_control_out_ is the output motor control for each joint.
     * Feeding this signal *IS MANDATORY* otherwize the process will crash.
     */
    dynamicgraph::Signal<dg::Vector,int> motor_control_out_;

    /**
     * @brief previous_motor_control_out_ is the last output motor control for
     * each joint. Feeding this signal *IS NOT* mandatory.
     */
    dynamicgraph::Signal<dg::Vector,int> previous_motor_control_out_;

    /**
     * @brief robot_state_ is the output state in generalized coodinates.
     * This signal is not requiered but helpful.
     */
    dynamicgraph::Signal<dg::Vector, int> robot_state_;

    /**
     * @brief robot_velocity_ is the output velocity in generalized coodinates.
     * This signal is not requiered but helpful.
     */
    dynamicgraph::Signal<dg::Vector, int> robot_velocity_;

    /**
     * @brief wrenchs_out_ is the list of the output 6d wrench data.
     */
    std::vector<dynamicgraph::Signal<dg::Vector,int> > wrenchs_out_;

  public:
    /************
     * COMMANDS *
     ************/
    /**
     * @brief commandLine I am not sure about this...
     * (was not virtual originally)
     */
    virtual void commandLine(const std::string&, std::istringstream&,
                     std::ostream&){}

  protected:
    /**
     * @brief periodic_call_before_ handle the asynchronous command call on the
     * device between getting the sensor data and sending the commands
     */
    PeriodicCall periodic_call_before_;

    /**
     * @brief periodic_call_after_ handle the asynchronous command call on the
     * device between getting the sensor data and sending the commands
     */
    PeriodicCall periodic_call_after_;

  };

} // namespace dynamic_graph


#endif /* #ifndef DEVICE_HH */




