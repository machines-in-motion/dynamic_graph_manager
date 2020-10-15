/**
 * @file ros_subscribe.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include "dynamic_graph_manager/ros_entities/ros_subscribe.hpp"

#include <dynamic-graph/factory.h>

#include "dynamic_graph_manager/dynamic_graph_manager.hpp"

namespace dynamic_graph_manager
{
/*
 * RosSubscribe
 */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosSubscribe, "RosSubscribe");

const std::string RosSubscribe::doc_string_(
    "Subscribe to a ROS topics and convert it into a dynamic-graph signals.\n"
    "\n"
    "  Use command \"add\" to subscribe to a new signal.\n");

RosSubscribe::RosSubscribe(const std::string& n) : dynamicgraph::Entity(n)
{
    ros_node_ = get_ros_node(DG_ROS_NODE_NAME);
    binded_signals_.clear();
    std::string doc_string =
        "\n"
        "  Add a signal reading data from a ROS topic.\n"
        "\n"
        "  Input:\n"
        "    - type: string among:\n";
    for (unsigned int i = 0; i < DgRosTypes::type_list.size(); ++i)
    {
        doc_string += "        - " + DgRosTypes::type_list[i] + "\n";
    }
    doc_string +=
        "    - signal: the signal name in dynamic-graph,\n"
        "    - topic:  the topic name in ROS.\n"
        "\n";
    addCommand("add", new command::ros_subscribe::Add(*this, doc_string));
    doc_string =
        "\n"
        "  Remove a signal reading data from a ROS topic\n"
        "\n"
        "  Input:\n"
        "    - name of the signal to remove (see method list for the list of "
        "signals).\n"
        "\n";
    addCommand("rm", new command::ros_subscribe::Rm(*this, doc_string));
    doc_string =
        "\n"
        "  Remove all signals reading data from a ROS topic\n"
        "\n"
        "  No input:\n"
        "\n";
    addCommand("clear", new command::ros_subscribe::Clear(*this, doc_string));
    doc_string =
        "\n"
        "  List signals reading data from a ROS topic\n"
        "\n"
        "  No input:\n"
        "\n";
    addCommand("list", new command::ros_subscribe::List(*this, doc_string));
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
    binded_signals_.erase(signal);

    if (binded_signals_.find(signalTs) != binded_signals_.end())
    {
        signalDeregistration(signalTs);
        binded_signals_.erase(signalTs);
    }
}

std::string RosSubscribe::list()
{
    std::string result("[");
    for (std::map<std::string, BindedSignal>::const_iterator it =
             binded_signals_.begin();
         it != binded_signals_.end();
         it++)
    {
        result += "'" + it->first + "',";
    }
    result += "]";
    return result;
}

void RosSubscribe::clear()
{
    std::map<std::string, BindedSignal>::iterator it =
        binded_signals_.begin();
    for (; it != binded_signals_.end();)
    {
        rm(it->first);
        it = binded_signals_.begin();
    }
}

std::string RosSubscribe::getDocString() const
{
    return doc_string_;
}

/*
 * Commands
 */

namespace command
{
namespace ros_subscribe
{
Clear::Clear(RosSubscribe& entity, const std::string& doc_string)
    : Command(entity, std::vector<Value::Type>(), doc_string)
{
}

Value Clear::doExecute()
{
    RosSubscribe& entity = static_cast<RosSubscribe&>(owner());

    entity.clear();
    return Value();
}

List::List(RosSubscribe& entity, const std::string& doc_string)
    : Command(entity, std::vector<Value::Type>(), doc_string)
{
}

Value List::doExecute()
{
    RosSubscribe& entity = static_cast<RosSubscribe&>(owner());
    return Value(entity.list());
}

Add::Add(RosSubscribe& entity, const std::string& doc_string)
    : Command(
          entity,
          boost::assign::list_of(Value::STRING)(Value::STRING)(Value::STRING),
          doc_string)
{
}

Value Add::doExecute()
{
    RosSubscribe& entity = static_cast<RosSubscribe&>(owner());
    std::vector<Value> values = getParameterValues();

    const std::string& type = values[0].value();
    const std::string& signal_name = values[1].value();
    const std::string& topic_name = values[2].value();

    if (type == DgRosTypes::get_double())
    {
        entity.add<std_msgs::msg::Float64, double>(signal_name, topic_name);
    }
    else if (type == DgRosTypes::get_unsigned())
    {
        entity.add<std_msgs::msg::UInt32, unsigned int>(signal_name,
                                                        topic_name);
    }
    else if (type == DgRosTypes::get_matrix())
    {
        entity.add<RosMatrix, DgMatrix>(signal_name, topic_name);
    }
    else if (type == DgRosTypes::get_vector())
    {
        entity.add<RosVector, DgVector>(signal_name, topic_name);
    }
    else if (type == DgRosTypes::get_vector3())
    {
        entity.add<geometry_msgs::msg::Vector3, DgVector>(signal_name,
                                                          topic_name);
    }
    else if (type == DgRosTypes::get_vector3_stamped())
    {
        entity.add<geometry_msgs::msg::Vector3Stamped, DgVector>(signal_name,
                                                                 topic_name);
    }
    else if (type == DgRosTypes::get_matrix_homogeneous())
    {
        entity.add<geometry_msgs::msg::Transform, MatrixHomogeneous>(
            signal_name, topic_name);
    }
    else if (type == DgRosTypes::get_matrix_homogeneous_stamped())
    {
        entity.add<geometry_msgs::msg::TransformStamped, MatrixHomogeneous>(
            signal_name, topic_name);
    }
    else if (type == DgRosTypes::get_twist())
    {
        entity.add<geometry_msgs::msg::Twist, DgVector>(signal_name,
                                                        topic_name);
    }
    else if (type == DgRosTypes::get_twist_stamped())
    {
        entity.add<geometry_msgs::msg::TwistStamped, DgVector>(signal_name,
                                                               topic_name);
    }
    else
    {
        std::cerr << "RosSubscribe(" << entity.getName()
                  << ")::add(): bad type given (" << type << ")."
                  << " Possible choice is among:";
        for (unsigned int i = 0; i < DgRosTypes::type_list.size(); ++i)
        {
            std::cerr << "    - " << DgRosTypes::type_list[i] << std::endl;
        }
    }
    return Value();
}

Rm::Rm(RosSubscribe& entity, const std::string& doc_string)
    : Command(entity, boost::assign::list_of(Value::STRING), doc_string)
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
}  // namespace ros_subscribe
}  // end of namespace command.

}  // namespace dynamic_graph_manager
