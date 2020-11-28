/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Define the time stamp signal.
 */

#pragma once

#include <dynamic-graph/signal-caster.h>

#include <chrono>

namespace dynamic_graph_manager
{

/** @brief Time stamp type. */
typedef std::chrono::time_point<std::chrono::high_resolution_clock> timestamp_t;

}  // namespace dynamic_graph_manager

namespace dynamicgraph
{
/**
 * @brief out stream the time stamp data.
 *
 * @param os
 * @param time_stamp
 * @return std::ostream&
 */
inline std::ostream &operator<<(
    std::ostream &os, const dynamic_graph_manager::timestamp_t &time_stamp)
{
    std::chrono::time_point<std::chrono::high_resolution_clock,
                            std::chrono::milliseconds>
        time_stamp_nanosec =
            std::chrono::time_point_cast<std::chrono::milliseconds>(time_stamp);
    os << time_stamp_nanosec.time_since_epoch().count();
    return os;
}

/**
 * @brief Structure used to serialize/deserialize the time stamp.
 *
 * @tparam
 */
template <>
struct signal_io<dynamic_graph_manager::timestamp_t>
    : signal_io_base<dynamic_graph_manager::timestamp_t>
{
    inline static dynamic_graph_manager::timestamp_t cast(std::istringstream &)
    {
        throw std::logic_error("this cast is not implemented.");
    }
};

}  // namespace dynamicgraph
