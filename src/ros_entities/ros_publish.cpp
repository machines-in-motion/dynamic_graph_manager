/**
 * @file ros_publish.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <stdexcept>

#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>

#include <dynamic-graph/command.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/linear-algebra.h>

#include "dynamic_graph_manager/dynamic_graph_manager.hpp"
#include "dynamic_graph_manager/ros_init.hpp"

#include "ros_entities/ros_publish.hpp"

namespace dynamic_graph_manager
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosPublish, "RosPublish");
const double RosPublish::ROS_JOINT_STATE_PUBLISHER_RATE = 0.01;

namespace command
{
namespace rosPublish
{
Clear::Clear(RosPublish& entity, const std::string& docstring)
    : Command(entity, std::vector<Value::Type>(), docstring)
{
}

Value Clear::doExecute()
{
    RosPublish& entity = static_cast<RosPublish&>(owner());

    entity.clear();
    return Value();
}

List::List(RosPublish& entity, const std::string& docstring)
    : Command(entity, std::vector<Value::Type>(), docstring)
{
}

Value List::doExecute()
{
    RosPublish& entity = static_cast<RosPublish&>(owner());
    return Value(entity.list());
}

Add::Add(RosPublish& entity, const std::string& docstring)
    : Command(
          entity,
          boost::assign::list_of(Value::STRING)(Value::STRING)(Value::STRING),
          docstring)
{
}

Value Add::doExecute()
{
    RosPublish& entity = static_cast<RosPublish&>(owner());
    std::vector<Value> values = getParameterValues();

    const std::string& type = values[0].value();
    const std::string& signal = values[1].value();
    const std::string& topic = values[2].value();

    if (type == "double")
        entity.add<double>(signal, topic);
    else if (type == "unsigned")
        entity.add<unsigned int>(signal, topic);
    else if (type == "matrix")
        entity.add<DgMatrix>(signal, topic);
    else if (type == "vector")
        entity.add<DgVector>(signal, topic);
    else if (type == "vector3")
        entity.add<specific::Vector3>(signal, topic);
    else if (type == "vector3Stamped")
        entity.add<std::pair<specific::Vector3, DgVector> >(signal, topic);
    else if (type == "matrixHomo")
        entity.add<MatrixHomogeneous>(signal, topic);
    else if (type == "matrixHomoStamped")
        entity.add<std::pair<MatrixHomogeneous, DgVector> >(signal, topic);
    else if (type == "twist")
        entity.add<specific::Twist>(signal, topic);
    else if (type == "twistStamped")
        entity.add<std::pair<specific::Twist, DgVector> >(signal, topic);
    else
        throw std::runtime_error("bad type");
    return Value();
}

Rm::Rm(RosPublish& entity, const std::string& docstring)
    : Command(entity, boost::assign::list_of(Value::STRING), docstring)
{
}

Value Rm::doExecute()
{
    RosPublish& entity = static_cast<RosPublish&>(owner());
    std::vector<Value> values = getParameterValues();
    const std::string& signal = values[0].value();
    entity.rm(signal);
    return Value();
}
}  // namespace rosPublish
}  // end of namespace command.

const std::string RosPublish::docstring_(
    "Publish dynamic-graph signals as ROS topics.\n"
    "\n"
    "  Use command \"add\" to publish a new ROS topic.\n");

RosPublish::RosPublish(const std::string& n)
    : dynamicgraph::Entity(n),
      // RosPublish do not use callback so do not create a useless spinner.
      nh_(ros_init(DynamicGraphManager::dg_ros_node_name_, true)),
      bindedSignal_(),
      trigger_(boost::bind(&RosPublish::trigger, this, _1, _2),
               dynamicgraph::sotNOSIGNAL,
               MAKE_SIGNAL_STRING(name, true, "int", "trigger")),
      rate_(ROS_JOINT_STATE_PUBLISHER_RATE),
      lastPublicated_()
{
    try
    {
        lastPublicated_ = ros::Time::now();
    }
    catch (const std::exception& exc)
    {
        throw std::runtime_error("Failed to call ros::Time::now ():" +
                                 std::string(exc.what()));
    }
    signalRegistration(trigger_);
    trigger_.setNeedUpdateFromAllChildren(true);

    std::string docstring =
        "\n"
        "  Add a signal writing data to a ROS topic\n"
        "\n"
        "  Input:\n"
        "    - type: string among ['double', 'matrix', 'vector', 'vector3',\n"
        "                          'vector3Stamped', 'matrixHomo', "
        "'matrixHomoStamped',\n"
        "                          'twist', 'twistStamped'],\n"
        "    - signal: the signal name in dynamic-graph,\n"
        "    - topic:  the topic name in ROS.\n"
        "\n";
    addCommand("add", new command::rosPublish::Add(*this, docstring));
    docstring =
        "\n"
        "  Remove a signal writing data to a ROS topic\n"
        "\n"
        "  Input:\n"
        "    - name of the signal to remove (see method list for the list of "
        "signals).\n"
        "\n";
    addCommand("rm", new command::rosPublish::Rm(*this, docstring));
    docstring =
        "\n"
        "  Remove all signals writing data to a ROS topic\n"
        "\n"
        "  No input:\n"
        "\n";
    addCommand("clear", new command::rosPublish::Clear(*this, docstring));
    docstring =
        "\n"
        "  List signals writing data to a ROS topic\n"
        "\n"
        "  No input:\n"
        "\n";
    addCommand("list", new command::rosPublish::List(*this, docstring));
}

RosPublish::~RosPublish()
{
}

void RosPublish::display(std::ostream& os) const
{
    os << CLASS_NAME << std::endl;
}

void RosPublish::rm(const std::string& signal)
{
    if (bindedSignal_.find(signal) == bindedSignal_.end()) return;

    if (signal == "trigger")
    {
        std::cerr << "The trigger signal should not be removed. Aborting."
                  << std::endl;
        return;
    }

    // lock the mutex to avoid deleting the signal during a call to trigger
    while (!mutex_.try_lock())
    {
    }
    signalDeregistration(signal);
    bindedSignal_.erase(signal);
    mutex_.unlock();
}

std::string RosPublish::list() const
{
    std::string result("[");
    for (std::map<std::string, bindedSignal_t>::const_iterator it =
             bindedSignal_.begin();
         it != bindedSignal_.end();
         it++)
    {
        result += "'" + it->first + "',";
    }
    result += "]";
    return result;
}

void RosPublish::clear()
{
    std::map<std::string, bindedSignal_t>::iterator it = bindedSignal_.begin();
    for (; it != bindedSignal_.end();)
    {
        if (it->first != "trigger")
        {
            rm(it->first);
            it = bindedSignal_.begin();
        }
        else
        {
            ++it;
        }
    }
}

int& RosPublish::trigger(int& dummy, int t)
{
    typedef std::map<std::string, bindedSignal_t>::iterator iterator_t;

    ros::Duration dt = ros::Time::now() - lastPublicated_;
    if (dt < rate_) return dummy;

    lastPublicated_ = ros::Time::now();

    while (!mutex_.try_lock())
    {
    }
    for (iterator_t it = bindedSignal_.begin(); it != bindedSignal_.end(); ++it)
    {
        boost::get<1>(it->second)(t);
    }
    mutex_.unlock();
    return dummy;
}

std::string RosPublish::getDocString() const
{
    return docstring_;
}

}  // namespace dynamic_graph_manager
