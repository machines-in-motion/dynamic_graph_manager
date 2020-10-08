/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements the template method of the RosPublish class.
 */

#pragma once

#include <vector>

#include "dynamic_graph_manager/ros_entities/dg_ros_mapping.hpp"

namespace dynamic_graph_manager
{
template <class ROS_TYPE, class DG_TYPE>
void RosPublish::send_data(
    std::shared_ptr<rclcpp::Publisher<
        typename DgRosMapping<ROS_TYPE, DG_TYPE>::ros_t> > publisher,
    std::shared_ptr<typename DgRosMapping<ROS_TYPE, DG_TYPE>::signal_in_t>
        signal)
{
    typename DgRosMapping<ROS_TYPE, DG_TYPE>::ros_t msg;
    DgRosMapping<ROS_TYPE, DG_TYPE>::dg_to_ros(signal->accessCopy(), msg);
    publisher->publish(msg);
}

template <typename ROS_TYPE, typename DG_TYPE>
void RosPublish::add(const std::string& signal, const std::string& topic)
{
    using ros_t = typename DgRosMapping<ROS_TYPE, DG_TYPE>::ros_t;
    using signal_in_t = typename DgRosMapping<ROS_TYPE, DG_TYPE>::signal_in_t;

    // Initialize the bindedSignal object.
    BindedSignal binded_signal;

    // Initialize the publisher.
    std::shared_ptr<rclcpp::Publisher<ros_t> > pub_ptr =
        ros_node_->create_publisher<ros_t>(topic, 10);

    // Initialize the signal.
    std::shared_ptr<signal_in_t> signal_ptr = std::make_shared<signal_in_t>(
            nullptr,  // no explicit dependence
            make_signal_string(
                *this,
                true,
                DgRosMapping<ROS_TYPE, DG_TYPE>::signal_type_name,
                signal));
    std::get<0>(binded_signal) = signal_ptr;
    DgRosMapping<ROS_TYPE, DG_TYPE>::set_default(signal_ptr);
    signalRegistration(*std::get<0>(binded_signal));

    // Initialize the callback.
    PublisherCallback callback = std::bind(
        &RosPublish::send_data<ROS_TYPE, DG_TYPE>, this, pub_ptr, signal_ptr);
    std::get<1>(binded_signal) = callback;

    binded_signals_[signal] = binded_signal;
}

}  // namespace dynamic_graph_manager
