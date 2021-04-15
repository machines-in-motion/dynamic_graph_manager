/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements the RosPublish class.
 */

#include "dynamic_graph_manager/ros_entities/ros_publish.hpp"

#include "dynamic_graph_manager/ros.hpp"

namespace dynamic_graph_manager
{
/*
 * RosPublish class
 */

const std::string RosPublish::doc_string_ =
    "Publish dynamic-graph signals as ROS topics.\n"
    "\n"
    "  Use command \"add\" to publish a new ROS topic.\n";

const std::string RosPublish::trigger_signal_name_ = "trigger";

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosPublish, "RosPublish");

RosPublish::RosPublish(const std::string& n)
    : dynamicgraph::Entity(n),
      trigger_(boost::bind(&RosPublish::trigger, this, _1, _2),
               dynamicgraph::sotNOSIGNAL,
               make_signal_string(*this, true, "int", trigger_signal_name_)),
      rate_nanosec_(std::chrono::nanoseconds(50000000))  // 50ms
{
    signalRegistration(trigger_);

    ros_node_ = get_ros_node(DG_ROS_NODE_NAME);
    ros_add_node_to_executor(DG_ROS_NODE_NAME);
    ros_spin_non_blocking();
    binded_signals_.clear();
    /** @todo Check this needUpdateFromAllChildren */
    trigger_.setNeedUpdateFromAllChildren(true);
    try
    {
        last_publicated_ = ros_node_->get_clock()->now();
    }
    catch (const std::exception& exc)
    {
        throw std::runtime_error(
            "Failed to call ros_node_->get_clock()->now():" +
            std::string(exc.what()));
    }
    mutex_.unlock();

    std::string doc_string;
    doc_string =
        "\n"
        "  Add a signal writing data to a ROS topic.\n"
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
    addCommand("add", new command::ros_publish::Add(*this, doc_string));
    doc_string =
        "\n"
        "  Remove a signal writing data to a ROS topic.\n"
        "\n"
        "  Input:\n"
        "    - name of the signal to remove (see method list for the list of "
        "signals).\n"
        "\n";
    addCommand("rm", new command::ros_publish::Rm(*this, doc_string));
    doc_string =
        "\n"
        "  Remove all signals writing data to a ROS topic.\n"
        "\n"
        "  No input.\n"
        "\n";
    addCommand("clear", new command::ros_publish::Clear(*this, doc_string));
    doc_string =
        "\n"
        "  List signals writing data to a ROS topic.\n"
        "\n"
        "  No input.\n"
        "\n";
    addCommand("list", new command::ros_publish::List(*this, doc_string));
}  // namespace dynamic_graph_manager

RosPublish::~RosPublish()
{
}

void RosPublish::display(std::ostream& os) const
{
    os << this->CLASS_NAME << "(" << this->name << ")." << std::endl;
}

void RosPublish::rm(const std::string& signal)
{
    if (binded_signals_.find(signal) == binded_signals_.end())
    {
        return;
    }

    if (signal == trigger_signal_name_)
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
    binded_signals_.erase(signal);
    mutex_.unlock();
}

std::string RosPublish::list() const
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

void RosPublish::clear()
{
    std::map<std::string, BindedSignal>::iterator it = binded_signals_.begin();
    for (; it != binded_signals_.end();)
    {
        if (it->first != trigger_signal_name_)
        {
            rm(it->first);
            it = binded_signals_.begin();
        }
        else
        {
            ++it;
        }
    }
}

int& RosPublish::trigger(int& dummy, int)
{
    typedef std::map<std::string, BindedSignal>::iterator iterator_t;

    // If we did not wait long enough (> rate_nanosec_), we do nothing.
    rclcpp::Duration dt = ros_node_->get_clock()->now() - last_publicated_;
    if (dt < rate_nanosec_)
    {
        return dummy;
    }

    // We reset the previous publishing time as "now".
    last_publicated_ = ros_node_->get_clock()->now();

    // We lock the mutex for the full duration of the publication.
    mutex_.lock();
    for (iterator_t it = binded_signals_.begin(); it != binded_signals_.end();
         ++it)
    {
        // calling the callback corresponding to the input signal in the tuple.
        std::get<1>(it->second)();
    }
    mutex_.unlock();
    return dummy;
}

std::string RosPublish::getDocString() const
{
    return doc_string_;
}

/*
 * RosPublish command
 */
namespace command
{
namespace ros_publish
{
Clear::Clear(RosPublish& entity, const std::string& doc_string)
    : Command(entity, std::vector<Value::Type>(), doc_string)
{
}

Value Clear::doExecute()
{
    RosPublish& entity = static_cast<RosPublish&>(owner());

    entity.clear();
    return Value();
}

List::List(RosPublish& entity, const std::string& doc_string)
    : Command(entity, std::vector<Value::Type>(), doc_string)
{
}

Value List::doExecute()
{
    RosPublish& entity = static_cast<RosPublish&>(owner());
    return Value(entity.list());
}

Add::Add(RosPublish& entity, const std::string& doc_string)
    : Command(
          entity,
          boost::assign::list_of(Value::STRING)(Value::STRING)(Value::STRING),
          doc_string)
{
}

Value Add::doExecute()
{
    RosPublish& entity = static_cast<RosPublish&>(owner());
    std::vector<Value> values = getParameterValues();

    const std::string& type = values[0].value();
    const std::string& signal = values[1].value();
    const std::string& topic = values[2].value();

    if (type == DgRosTypes::get_double())
    {
        entity.add<std_msgs::msg::Float64, double>(signal, topic);
    }
    else if (type == DgRosTypes::get_unsigned())
    {
        entity.add<std_msgs::msg::UInt32, unsigned int>(signal, topic);
    }
    else if (type == DgRosTypes::get_matrix())
    {
        entity.add<RosMatrix, DgMatrix>(signal, topic);
    }
    else if (type == DgRosTypes::get_vector())
    {
        entity.add<RosVector, DgVector>(signal, topic);
    }
    else if (type == DgRosTypes::get_vector3())
    {
        entity.add<geometry_msgs::msg::Vector3, DgVector>(signal, topic);
    }
    else if (type == DgRosTypes::get_vector3_stamped())
    {
        entity.add<geometry_msgs::msg::Vector3Stamped, DgVector>(signal, topic);
    }
    else if (type == DgRosTypes::get_matrix_homogeneous())
    {
        entity.add<geometry_msgs::msg::Transform, MatrixHomogeneous>(signal,
                                                                     topic);
    }
    else if (type == DgRosTypes::get_matrix_homogeneous_stamped())
    {
        entity.add<geometry_msgs::msg::TransformStamped, MatrixHomogeneous>(
            signal, topic);
    }
    else if (type == DgRosTypes::get_twist())
    {
        entity.add<geometry_msgs::msg::Twist, DgVector>(signal, topic);
    }
    else if (type == DgRosTypes::get_twist_stamped())
    {
        entity.add<geometry_msgs::msg::TwistStamped, DgVector>(signal, topic);
    }
    else
    {
        std::cerr << "RosPublish(" << entity.getName()
                  << ")::add(): bad type given (" << type << ")."
                  << " Possible choice is among:";
        for (unsigned int i = 0; i < DgRosTypes::type_list.size(); ++i)
        {
            std::cerr << "    - " << DgRosTypes::type_list[i] << std::endl;
        }
    }
    return Value();
}

Rm::Rm(RosPublish& entity, const std::string& doc_string)
    : Command(entity, boost::assign::list_of(Value::STRING), doc_string)
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
}  // namespace ros_publish
}  // end of namespace command.

}  // namespace dynamic_graph_manager
