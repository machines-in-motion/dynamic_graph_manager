/**
 * @file ros_subscribe.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>

#include <dynamic-graph/factory.h>

#include "dynamic_graph_manager/dynamic_graph_manager.hpp"
#include "dynamic_graph_manager/ros.hpp"

#include "dynamic_graph_manager/ros_entities/ros_subscribe.hpp"

namespace dynamic_graph_manager
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosSubscribe, "RosSubscribe");

namespace command
{
namespace rosSubscribe
{
Clear::Clear(RosSubscribe& entity, const std::string& docstring)
    : Command(entity, std::vector<Value::Type>(), docstring)
{
}

Value Clear::doExecute()
{
    RosSubscribe& entity = static_cast<RosSubscribe&>(owner());

    entity.clear();
    return Value();
}

List::List(RosSubscribe& entity, const std::string& docstring)
    : Command(entity, std::vector<Value::Type>(), docstring)
{
}

Value List::doExecute()
{
    RosSubscribe& entity = static_cast<RosSubscribe&>(owner());
    return Value(entity.list());
}

Add::Add(RosSubscribe& entity, const std::string& docstring)
    : Command(
          entity,
          boost::assign::list_of(Value::STRING)(Value::STRING)(Value::STRING),
          docstring)
{
}

Value Add::doExecute()
{
    RosSubscribe& entity = static_cast<RosSubscribe&>(owner());
    std::vector<Value> values = getParameterValues();

    const std::string& type = values[0].value();
    const std::string& signal = values[1].value();
    const std::string& topic = values[2].value();

    if (type == "double")
        entity.add<double>(signal, topic);
    else if (type == "unsigned")
        entity.add<unsigned int>(signal, topic);
    else if (type == "matrix")
        entity.add<dg::Matrix>(signal, topic);
    else if (type == "vector")
        entity.add<dg::Vector>(signal, topic);
    else if (type == "vector3")
        entity.add<specific::Vector3>(signal, topic);
    else if (type == "vector3Stamped")
        entity.add<std::pair<specific::Vector3, dg::Vector> >(signal, topic);
    else if (type == "matrixHomo")
        entity.add<MatrixHomogeneous>(signal, topic);
    else if (type == "matrixHomoStamped")
        entity.add<std::pair<MatrixHomogeneous, dg::Vector> >(signal, topic);
    else if (type == "twist")
        entity.add<specific::Twist>(signal, topic);
    else if (type == "twistStamped")
        entity.add<std::pair<specific::Twist, dg::Vector> >(signal, topic);
    else
        throw std::runtime_error("bad type");
    return Value();
}

Rm::Rm(RosSubscribe& entity, const std::string& docstring)
    : Command(entity, boost::assign::list_of(Value::STRING), docstring)
{
}

Value Rm::doExecute()
{
    RosSubscribe& entity = static_cast<RosSubscribe&>(owner());
    std::vector<Value> values = getParameterValues();
    const std::string& signal = values[0].value();
    entity.rm(signal);
    return Value();
}
}  // namespace rosSubscribe
}  // end of namespace command.

const std::string RosSubscribe::docstring_(
    "Subscribe to a ROS topics and convert it into a dynamic-graph signals.\n"
    "\n"
    "  Use command \"add\" to subscribe to a new signal.\n");

RosSubscribe::RosSubscribe(const std::string& n)
    : dynamicgraph::Entity(n),
      nh_(ros_init(DynamicGraphManager::dg_ros_node_name_, true)),
      bindedSignal_()
{
    std::string docstring =
        "\n"
        "  Add a signal reading data from a ROS topic\n"
        "\n"
        "  Input:\n"
        "    - type: string among ['double', 'matrix', 'vector', 'vector3',\n"
        "                          'vector3Stamped', 'matrixHomo', "
        "'matrixHomoStamped',\n"
        "                          'twist', 'twistStamped'],\n"
        "    - signal: the signal name in dynamic-graph,\n"
        "    - topic:  the topic name in ROS.\n"
        "\n";
    addCommand("add", new command::rosSubscribe::Add(*this, docstring));
    docstring =
        "\n"
        "  Remove a signal reading data from a ROS topic\n"
        "\n"
        "  Input:\n"
        "    - name of the signal to remove (see method list for the list of "
        "signals).\n"
        "\n";
    addCommand("rm", new command::rosSubscribe::Rm(*this, docstring));
    docstring =
        "\n"
        "  Remove all signals reading data from a ROS topic\n"
        "\n"
        "  No input:\n"
        "\n";
    addCommand("clear", new command::rosSubscribe::Clear(*this, docstring));
    docstring =
        "\n"
        "  List signals reading data from a ROS topic\n"
        "\n"
        "  No input:\n"
        "\n";
    addCommand("list", new command::rosSubscribe::List(*this, docstring));
}

RosSubscribe::~RosSubscribe()
{
    clear();
}

void RosSubscribe::display(std::ostream& os) const
{
    os << CLASS_NAME << std::endl;
}

void RosSubscribe::rm(const std::string& signal)
{
    std::string signalTs = signal + "Timestamp";

    signalDeregistration(signal);
    bindedSignal_.erase(signal);

    if (bindedSignal_.find(signalTs) != bindedSignal_.end())
    {
        signalDeregistration(signalTs);
        bindedSignal_.erase(signalTs);
    }
}

std::string RosSubscribe::list()
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

void RosSubscribe::clear()
{
    std::map<std::string, bindedSignal_t>::iterator it = bindedSignal_.begin();
    for (; it != bindedSignal_.end();)
    {
        rm(it->first);
        it = bindedSignal_.begin();
    }
}

std::string RosSubscribe::getDocString() const
{
    return docstring_;
}
}  // namespace dynamic_graph_manager
