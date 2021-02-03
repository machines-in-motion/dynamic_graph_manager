/**
 * @file ros_subscribe.hxx
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#pragma once

#include <message_filters/message_traits.h>

#include "dynamic_graph_manager/ros_entities/dg_ros_mapping.hpp"

namespace dynamic_graph_manager
{
template <typename RosType, typename DgType>
void RosSubscribe::callback(
    std::shared_ptr<typename DgRosMapping<RosType, DgType>::signal_out_t>
        signal_out,
    std::shared_ptr<
        typename DgRosMapping<RosType, DgType>::signal_timestamp_out_t>
        signal_timestamp_out,
    const std::shared_ptr<typename DgRosMapping<RosType, DgType>::ros_t>
        ros_data)
{
    // Some convenient shortcut.
    using ros_t = typename DgRosMapping<RosType, DgType>::ros_t;
    using dg_t = typename DgRosMapping<RosType, DgType>::dg_t;

    // Create a dynamic graph object to copy the information in. Not real time
    // safe?
    dg_t value;
    // Convert the ROS topic into a dynamic graph type.
    DgRosMapping<RosType, DgType>::ros_to_dg(*ros_data, value);
    // Update the signal.
    signal_out->setConstant(value);
    // If he ROS message has a header then copy the header into the timestamp
    // signal
    if (signal_timestamp_out)
    {
        signal_timestamp_out->setConstant(
            DgRosMapping<RosType, DgType>::from_ros_time(
                message_filters::message_traits::TimeStamp<ros_t>::value(
                    *ros_data)));
    }
}

template <typename RosType, typename DgType>
void RosSubscribe::add(const std::string& signal_name,
                       const std::string& topic_name)
{
    using dg_t = typename DgRosMapping<RosType, DgType>::dg_t;
    using ros_t = typename DgRosMapping<RosType, DgType>::ros_t;
    using signal_out_t = typename DgRosMapping<RosType, DgType>::signal_out_t;
    using signal_timestamp_out_t =
        typename DgRosMapping<RosType, DgType>::signal_timestamp_out_t;

    if (binded_signals_.find(signal_name) != binded_signals_.end())
    {
        std::cout << "Signal already created, nothing to be done." << std::endl;
        return;
    }

    // Initialize the binded_signal object.
    RosSubscribe::BindedSignal binded_signal;

    // Initialize the signal.
    std::string full_signal_name =
        this->getClassName() + "(" + this->getName() + ")::" + signal_name;
    std::shared_ptr<signal_out_t> signal_ptr =
        std::make_shared<signal_out_t>(full_signal_name);
    DgRosMapping<RosType, DgType>::set_default(signal_ptr);
    signal_ptr->setDependencyType(
        dynamicgraph::TimeDependency<int>::ALWAYS_READY);
    std::get<0>(binded_signal) = signal_ptr;
    this->signalRegistration(*std::get<0>(binded_signal));

    // Initialize the time stamp signal if needed.
    std::shared_ptr<signal_timestamp_out_t> signal_timestamp_ptr = nullptr;
    if (message_filters::message_traits::HasHeader<ros_t>())
    {
        std::string full_time_stamp_signal_name =
            this->getClassName() + "(" + this->getName() + ")::" + signal_name +
            "_timestamp";
        signal_timestamp_ptr = std::make_shared<signal_timestamp_out_t>(
            full_time_stamp_signal_name);
        signal_timestamp_ptr->setConstant(
            DgRosMapping<RosType, DgType>::epoch_time());
        signal_timestamp_ptr->setDependencyType(
            dynamicgraph::TimeDependency<int>::ALWAYS_READY);
        this->signalRegistration(*signal_timestamp_ptr);
    }
    std::get<1>(binded_signal) = signal_timestamp_ptr;

    // Initialize the subscriber.
    std::function<void(const std::shared_ptr<ros_t>)> callback =
        std::bind(&RosSubscribe::callback<ros_t, dg_t>,
                  this,
                  signal_ptr,
                  signal_timestamp_ptr,
                  std::placeholders::_1);
    typename rclcpp::Subscription<ros_t>::SharedPtr sub =
        ros_node_->create_subscription<ros_t>(topic_name, 10, callback);
    std::get<2>(binded_signal) = sub;

    // Store the different pointers.
    binded_signals_[signal_name] = binded_signal;
}

}  // namespace dynamic_graph_manager
