/**
 * @file ros_publish.hxx
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef DYNAMIC_GRAPH_ROS_PUBLISH_HXX
#define DYNAMIC_GRAPH_ROS_PUBLISH_HXX
#include <std_msgs/Float64.h>
#include <vector>

#include "dynamic_graph_manager/Matrix.h"
#include "dynamic_graph_manager/Vector.h"

#include "dg_to_ros.hpp"

#include <iostream>

namespace dynamic_graph_manager
{
template <>
inline void RosPublish::sendData<std::pair<MatrixHomogeneous, DgVector> >(
    boost::shared_ptr<realtime_tools::RealtimePublisher<
        DgToRos<std::pair<MatrixHomogeneous, DgVector> >::ros_t> > publisher,
    boost::shared_ptr<
        DgToRos<std::pair<MatrixHomogeneous, DgVector> >::signalIn_t> signal,
    int time)
{
    DgToRos<std::pair<MatrixHomogeneous, DgVector> >::ros_t result;
    if (publisher->trylock())
    {
        publisher->msg_.child_frame_id = "/dynamic_graph/world";
        converter(publisher->msg_, signal->access(time));
        publisher->unlockAndPublish();
    }
}

template <typename T>
void RosPublish::sendData(
    boost::shared_ptr<realtime_tools::RealtimePublisher<
        typename DgToRos<T>::ros_t> > publisher,
    boost::shared_ptr<typename DgToRos<T>::signalIn_t> signal,
    int time)
{
    typename DgToRos<T>::ros_t result;
    if (publisher->trylock())
    {
        converter(publisher->msg_, signal->access(time));
        publisher->unlockAndPublish();
    }
}

template <typename T>
void RosPublish::add(const std::string& signal, const std::string& topic)
{
    typedef typename DgToRos<T>::ros_t ros_t;
    typedef typename DgToRos<T>::signalIn_t signal_t;

    // Initialize the bindedSignal object.
    bindedSignal_t bindedSignal;

    // Initialize the publisher.
    boost::shared_ptr<realtime_tools::RealtimePublisher<ros_t> > pubPtr =
        boost::make_shared<realtime_tools::RealtimePublisher<ros_t> >(
            nh_, topic, 1);

    // Initialize the signal.
    boost::shared_ptr<signal_t> signalPtr(new signal_t(
        0, MAKE_SIGNAL_STRING(name, true, DgToRos<T>::signalTypeName, signal)));
    boost::get<0>(bindedSignal) = signalPtr;
    DgToRos<T>::setDefault(*signalPtr);
    signalRegistration(*boost::get<0>(bindedSignal));

    // Initialize the callback.
    callback_t callback =
        boost::bind(&RosPublish::sendData<T>, this, pubPtr, signalPtr, _1);
    boost::get<1>(bindedSignal) = callback;

    bindedSignal_[signal] = bindedSignal;
}

}  // namespace dynamic_graph_manager

#endif  //! DYNAMIC_GRAPH_ROS_PUBLISH_HXX
