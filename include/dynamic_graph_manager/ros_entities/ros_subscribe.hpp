/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Entity that subscribe to rostopic and forward the data to the dynamic
 * graph trough a signal.
 */

#pragma once

#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <map>
#include <mutex>

#include "dynamic_graph_manager/ros_entities/dg_ros_mapping.hpp"

namespace dynamic_graph_manager
{
/**
 * @brief  Publish ROS information in the dynamic-graph.
 */
class RosSubscribe : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Tuple composed by the generated input signal and its callback
     * function. The callback function publishes the input signal content.
     */
    typedef std::tuple<std::shared_ptr<dynamicgraph::SignalBase<int> >,
                       std::shared_ptr<dynamicgraph::SignalBase<int> >,
                       std::shared_ptr<rclcpp::SubscriptionBase> >
        BindedSignal;

    /**
     * @brief Construct a new Ros Subscribe object
     *
     * @param name entity name.
     */
    RosSubscribe(const std::string& name);

    /**
     * @brief Destroy the Ros Subscribe object.
     */
    virtual ~RosSubscribe();

    /**
     * @brief Get Doc String.
     *
     * @return std::string
     */
    virtual std::string getDocString() const;

    /**
     * @brief Display information about the entity.
     *
     * @param os
     */
    void display(std::ostream& os) const;

    /**
     * @brief Subscribe to a topic and add a signal containing the topic data.
     *
     * @tparam RosType
     * @tparam DgType
     * @param signal_name
     * @param topic_name
     */
    template <typename RosType, typename DgType>
    void add(const std::string& signal_name, const std::string& topic_name);

    /**
     * @brief Unsubscribe to a topic and remove the signal.
     *
     * @param signal
     */
    void rm(const std::string& signal);

    /**
     * @brief List of all subscribed topic and singals.
     *
     * @return std::string
     */
    std::string list();

    /**
     * @brief Unsubscribe to all topics and remove all signals.
     *
     */
    void clear();

private:
    /**
     * @brief This callback feeds the data signal upon reception of a ROS
     * message.
     *
     * @tparam RosType
     * @tparam DgType
     * @param signal pointer to the dynamic graph signal.
     * @param data ROS data to copy from.
     */
    template <typename RosType, typename DgType>
    void callback(
        std::shared_ptr<typename DgRosMapping<RosType, DgType>::signal_out_t>
            signal_out,
        std::shared_ptr<
            typename DgRosMapping<RosType, DgType>::signal_timestamp_out_t>
            signal_timestamp_out,
        const std::shared_ptr<typename DgRosMapping<RosType, DgType>::ros_t>
            ros_data);

private:
    /** @brief Doc string associated to the entity. */
    static const std::string doc_string_;

    /** @brief ROS node pointer. */
    RosNodePtr ros_node_;

    /** @brief Named list of signals associated with it's callback functions. */
    std::map<std::string, BindedSignal> binded_signals_;
};

namespace command
{
namespace ros_subscribe
{
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

#define ROS_SUBSCRIBE_MAKE_COMMAND(CMD)                           \
    class CMD : public Command                                    \
    {                                                             \
    public:                                                       \
        CMD(RosSubscribe& entity, const std::string& doc_string); \
        virtual Value doExecute();                                \
    }

ROS_SUBSCRIBE_MAKE_COMMAND(Add);
ROS_SUBSCRIBE_MAKE_COMMAND(Clear);
ROS_SUBSCRIBE_MAKE_COMMAND(List);
ROS_SUBSCRIBE_MAKE_COMMAND(Rm);

#undef ROS_SUBSCRIBE_MAKE_COMMAND

}  // namespace ros_subscribe
}  // end of namespace command.

}  // namespace dynamic_graph_manager

#include "ros_subscribe.hxx"
