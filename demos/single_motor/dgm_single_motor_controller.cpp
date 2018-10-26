/**
 * This example is based on:
 * https://github.com/stack-of-tasks/dynamic-graph-tutorial/blob/master/src/feedback-controller.cpp
 */

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include "dgm_single_motor_controller.hh"

using namespace dynamic_graph_demo;

// Register new Entity type in the factory
// Note that the second argument is the type name of the python class
// that will be created when importing the python module.
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MotorController, "MotorController");

MotorController::MotorController(const std::string& inName) :
  dynamicgraph::Entity(inName),
  stateSIN(NULL, "MotorController("+inName+")::input(vector)::state"),
  torqueSOUT(stateSIN,
	    "MotorController("+inName+")::output(vector)::torque"),
  gain_(dynamicgraph::Matrix(1,1))
{
  // Register signals into the entity.
  signalRegistration(stateSIN);
  signalRegistration(torqueSOUT);

  // Set signals as constant to size them
  dynamicgraph::Vector state(1);
  state.fill(0.);
  dynamicgraph::Vector torque(1);
  torque.fill(0.);
  torqueSOUT.setConstant(torque);
  stateSIN.setConstant(state);
  desired_ = state;

  // Define refresh function for output signal
  torqueSOUT.setFunction(boost::bind(&MotorController::computeMotorControl,
				    this, _1, _2));
  std::string docstring;
  // setGain
  docstring =
    "\n"
    "    Set gain of controller\n"
    "      takes a tuple of 1 floating point numbers as input\n"
    "\n";
  addCommand(std::string("setGain"),
	     new ::dynamicgraph::command::Setter<MotorController, dynamicgraph::Matrix>
	     (*this, &MotorController::setGain, docstring));

  docstring =
    "\n"
    "    Set the desired position of the controller\n"
    "      takes a tuple of 1 floating point numbers as input\n"
    "\n";
  addCommand(std::string("setDesired"),
	     new ::dynamicgraph::command::Setter<MotorController, dynamicgraph::Matrix>
	     (*this, &MotorController::setDesired, docstring));

  // getGain
  docstring =
    "\n"
    "    Get gain of controller\n"
    "      return a tuple of 1 floating point numbers\n"
    "\n";
  addCommand(std::string("getGain"),
	     new ::dynamicgraph::command::Getter<MotorController, dynamicgraph::Matrix>
	     (*this, &MotorController::getGain, docstring));
}

MotorController::~MotorController()
{
}

dynamicgraph::Vector& MotorController::computeMotorControl(dynamicgraph::Vector& torque,
						 const int& inTime)
{
  const dynamicgraph::Vector& state = stateSIN(inTime);

  if (state.size() != 1)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 1.",
					state.size());
  dynamicgraph::Vector v (gain_ * (desired_ - state));
  // dynamicgraph::Vector v (gain_ () * state);
  torque = v;

  return torque;
}
