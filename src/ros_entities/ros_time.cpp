/**
 * @file ros_time.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-cast-helper.h>
#include <dynamic-graph/signal-caster.h>

#include "ros_entities/converter.hpp"
#include "ros_entities/ros_time.hpp"

namespace dynamic_graph
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTime, "RosTime");

using namespace boost::posix_time;

const std::string RosTime::docstring_(
    "Export ROS time into dynamic-graph.\n"
    "\n"
    "  Signal \"time\" provides time as given by ros::time as\n"
    "  boost::posix_time::ptime type.\n");

RosTime::RosTime(const std::string& name)
    : Entity(name),
      now_("RosTime(" + name + ")::output(boost::posix_time::ptime)::time")
{
    signalRegistration(now_);
    now_.setConstant(rosTimeToPtime(ros::Time::now()));
    now_.setFunction(boost::bind(&RosTime::update, this, _1, _2));
}

ptime& RosTime::update(ptime& time, const int&)
{
    time = rosTimeToPtime(ros::Time::now());
    return time;
}

std::string RosTime::getDocString() const
{
    return docstring_;
}
}  // namespace dynamic_graph
