/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Declare the RosPublish class.
 */

#pragma once

#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <iostream>
#include <map>
#include <mutex>

#include "dynamic_graph_manager/ros_entities/dg_ros_mapping.hpp"

namespace dynamic_graph_manager
{
/** @brief Publish dynamic-graph information into ROS. */
class RosPublish : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Callback function publishing in the ROS topic.
     * This callback is called during the trigger signal callback function.
     */
    typedef std::function<void(void)> PublisherCallback;

    /**
     * @brief Tuple composed by the generated input signal and its callback
     * function. The callback function publishes the input signal content.
     */
    typedef std::tuple<std::shared_ptr<dynamicgraph::SignalBase<int> >,
                       PublisherCallback>
        BindedSignal;

    /**
     * @brief Construct a new RosPublish object.
     *
     * @param n
     */
    RosPublish(const std::string& n);

    /**
     * @brief Destroy the RosPublish object.
     *
     */
    virtual ~RosPublish();

    /**
     * @brief Get the documentation string.
     *
     * @return std::string
     */
    virtual std::string getDocString() const;

    /**
     * @brief Display the class information.
     *
     * @param os
     */
    void display(std::ostream& os) const;

    /**
     * @brief Add a signal to publish to ROS.
     * 
     * @tparam ROS_TYPE type of the ROS message.
     * @tparam DG_TYPE type of the Dynamic Graph signal data.
     * @param signal name.
     * @param topic name.
     */
    template <typename ROS_TYPE, typename DG_TYPE>
    void add(const std::string& signal, const std::string& topic);

    /**
     * @brief Remove a signal to publish to ROS.
     *
     * @param signal name.
     */
    void rm(const std::string& signal);

    /**
     * @brief List of all signal and topics currently published.
     *
     * @return std::string
     */
    std::string list() const;

    /**
     * @brief Remove all signal published to ROS.
     */
    void clear();
    
private:
    /**
     * @brief Trigger signal callback method.
     *
     * @return int&
     */
    int& trigger(int&, int);


    /**
     * @brief Send the data from the input signal to the ROS topic.
     * 
     * @tparam ROS_TYPE ROS message type.
     * @tparam DG_TYPE Dynamic graph signal data type.
     * @param publisher pointer to the ros publisher.
     * @param signal signal name.
     */
    template <class ROS_TYPE, class DG_TYPE>
    void send_data(
        std::shared_ptr<rclcpp::Publisher<
            typename DgRosMapping<ROS_TYPE, DG_TYPE>::ros_t> > publisher,
        std::shared_ptr<typename DgRosMapping<ROS_TYPE, DG_TYPE>::signal_in_t>
            signal);

private:
    /** @brief Doc string associated to the entity. */
    static const std::string doc_string_;

    /** @brief Name of the trigger signal. */
    static const std::string trigger_signal_name_;

    /** @brief Trigger signal publishing the signal values every other
     * iterations. */
    dynamicgraph::SignalTimeDependent<int, int> trigger_;

    /** @brief Publishing rate. Default is 50ms. */
    rclcpp::Duration rate_nanosec_;

    /** @brief ROS node pointer. */
    RosNodePtr ros_node_;

    /** @brief Named list of signals associated with it's callback functions. */
    std::map<std::string, BindedSignal> binded_signals_;

    /** @brief Last published time. */
    rclcpp::Time last_publicated_;

    /** @brief Protects the command and trigger callbacks. */
    std::mutex mutex_;
};

/*
 * Commands
 */
namespace command
{
namespace ros_publish
{
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

#define ROS_PUBLISH_MAKE_COMMAND(CMD)                          \
    class CMD : public Command                                 \
    {                                                          \
    public:                                                    \
        CMD(RosPublish& entity, const std::string& docstring); \
        virtual Value doExecute();                             \
    }

ROS_PUBLISH_MAKE_COMMAND(Add);
ROS_PUBLISH_MAKE_COMMAND(Clear);
ROS_PUBLISH_MAKE_COMMAND(List);
ROS_PUBLISH_MAKE_COMMAND(Rm);

#undef ROS_PUBLISH_MAKE_COMMAND

}  // namespace ros_publish
}  // end of namespace command.

}  // namespace dynamic_graph_manager

#include "ros_publish.hxx"
