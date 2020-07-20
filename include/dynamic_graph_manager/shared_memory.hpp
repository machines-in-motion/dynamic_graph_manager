/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Informations on the shared memory usage.
 */

#pragma once

// used to deal with shared memory
#include "shared_memory/locked_condition_variable.hpp"

namespace dynamic_graph_manager
{
/**
 * @brief Information about the shared memory.
 * Notably the name of the shared objects and segment.
 */
namespace dgm_shared_memory
{
/** @brief Name of the shared memory segment. */
static const std::string shared_memory_name = "DGM_ShM";

/** @brief Name of the std::map containing the sensor data. */
static const std::string sensors_map_name = "sensors_map_name";

/** @brief Name of the std::map containing the joint control data. */
static const std::string joint_controls_map_name = "joint_controls_map_name";

/** @brief Name of the shared_memory::condition_variable. */
static const std::string cond_var_name = "cond_var_name";

/** @brief Clean the shared memory sgement used by the control architecture.
 */
static void clean()
{
    shared_memory::LockedConditionVariable::clean(cond_var_name);
    shared_memory::delete_segment(shared_memory_name);
}

}  // namespace dgm_shared_memory
}  // namespace dynamic_graph_manager
