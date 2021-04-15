/**
 * @file dg_to_ros.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */
#pragma once

// Standard includes
#include <sstream>
#include <utility>
#include <vector>

// Dynamic Graph types.
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

// ROS types
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>

// internal DG types
#include "dynamic_graph_manager/ros_entities/matrix_geometry.hpp"
#include "dynamic_graph_manager/ros_entities/time_point_io.hpp"

// internal ros types
#include "dynamic_graph_manager/ros.hpp"
#include "mim_msgs/msg/matrix.hpp"
#include "mim_msgs/msg/vector.hpp"

namespace dynamic_graph_manager
{
/** @brief Conventient renaming for ease of implementation. */
typedef dynamicgraph::Vector DgVector;
/** @brief Conventient renaming for ease of implementation. */
typedef dynamicgraph::Matrix DgMatrix;
/** @brief Conventient renaming for ease of implementation. */
typedef mim_msgs::msg::Vector RosVector;
/** @brief Conventient renaming for ease of implementation. */
typedef mim_msgs::msg::Matrix RosMatrix;

struct DgRosTypes
{
    static std::vector<std::string> create_type_list()
    {
        std::vector<std::string> tmp_type_list;
        tmp_type_list.push_back("double");
        tmp_type_list.push_back("unsigned");
        tmp_type_list.push_back("matrix");
        tmp_type_list.push_back("vector");
        tmp_type_list.push_back("vector3");
        tmp_type_list.push_back("vector3_stamped");
        tmp_type_list.push_back("matrix_homogeneous");
        tmp_type_list.push_back("matrix_homogeneous_stamped");
        tmp_type_list.push_back("twist");
        tmp_type_list.push_back("twist_stamped");
        return tmp_type_list;
    }
    // clang-format off
    static const std::string& get_double(){return type_list[0];};
    static const std::string& get_unsigned(){return type_list[1];};
    static const std::string& get_matrix(){return type_list[2];};
    static const std::string& get_vector(){return type_list[3];};
    static const std::string& get_vector3(){return type_list[4];};
    static const std::string& get_vector3_stamped(){return type_list[5];};
    static const std::string& get_matrix_homogeneous(){return type_list[6];};
    static const std::string& get_matrix_homogeneous_stamped()
        {return type_list[7];};
    static const std::string& get_twist(){return type_list[8];};
    static const std::string& get_twist_stamped(){return type_list[9];};
    // clang-format on
    static const std::vector<std::string> type_list;
};

/**
 * @brief DgRosMapping trait type.
 *
 * DG stands for Dynamic Graph and ROS for Robot Operating System.
 * This trait provides types associated to a dynamic-graph type:
 * - DG / ROS corresponding types,
 * - Output/Input signal type,
 * - ROS / DG callback type.
 *
 * @tparam RosType is the type of the ROS data.
 * @tparam DgType is the type of the Dynamic Graph data.
 */
template <class RosType, class DgType>
class DgRosMapping
{
public:
    /*
     * Some renaming first
     */

    /** @brief Dynamic Graph type. */
    typedef DgType dg_t;
    /** @brief Ros type. */
    typedef RosType ros_t;
    /** @brief Ros type as a shared pointer. */
    typedef std::shared_ptr<RosType> ros_shared_ptr_t;
    /** @brief Output signal type. */
    typedef dynamicgraph::SignalTimeDependent<dg_t, int> signal_out_t;
    /** @brief Output signal type. */
    typedef dynamicgraph::SignalTimeDependent<timestamp_t, int>
        signal_timestamp_out_t;
    /** @brief Input signal type. */
    typedef dynamicgraph::SignalPtr<dg_t, int> signal_in_t;
    /** @brief Signal callback function types. */
    typedef std::function<dg_t&(dg_t&, int)> signal_callback_t;

    /** @brief Name of the signal type. Used during the creation of signals. */
    static const std::string signal_type_name;
    static const dg_t default_value;

    /**
     * @brief Set the default value to the inuput DG signal.
     *
     * @tparam SignalType (in/out)
     * @param s
     */
    template <typename SignalTypePtr>
    static void set_default(SignalTypePtr s)
    {
        s->setConstant(DgRosMapping<RosType, DgType>::default_value);
    }

    /**
     * @brief Set the default value to the input DG object.
     *
     * @param s
     */
    static void set_default(dg_t& d)
    {
        d = DgRosMapping<RosType, DgType>::default_value;
    }

    /**
     * @brief Convert ROS time to std::chrono.
     *
     * @param ros_time
     * @return timestamp_t
     */
    static timestamp_t from_ros_time(rclcpp::Time ros_time)
    {
        return epoch_time() + std::chrono::nanoseconds(ros_time.nanoseconds());
    }

    /**
     * @brief Get epoch time as ROS time start from there.
     *
     * @return timestamp_t
     */
    static timestamp_t epoch_time()
    {
        return std::chrono::time_point<std::chrono::high_resolution_clock>{};
    }

    /**
     * @brief Convert a ROS object into DG one.
     *
     * @param src
     * @param dst
     */
    static void ros_to_dg(const ros_t& ros_src, dg_t& dg_dst);

    /**
     * @brief Convert a DG object into a ROS one.
     *
     * @param src
     * @param dst
     */
    static void dg_to_ros(const dg_t& dg_src, ros_t& ros_dst);
};

template <class RosEntity>
inline std::string make_signal_string(const RosEntity& entity,
                                      bool isInputSignal,
                                      const std::string& signal_type,
                                      const std::string& signal_name)
{
    std::ostringstream oss;
    oss << entity.getClassName() << "(" << entity.getName() << ")"
        << "::" << (isInputSignal ? "input" : "output") << "(" << signal_type
        << ")"
        << "::" << signal_name;
    return oss.str();
}

inline void make_header(std_msgs::msg::Header& header)
{
    header.stamp = get_ros_node(DG_ROS_NODE_NAME)->get_clock()->now();
    header.frame_id = "/dynamic_graph/world";
}

}  // namespace dynamic_graph_manager
