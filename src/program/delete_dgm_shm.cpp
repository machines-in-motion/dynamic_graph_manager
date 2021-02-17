/**
 * @file
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 *
 * @brief Purge the shared memory used by the dynamic_graph_manager
 */

#include "dynamic_graph_manager/dynamic_graph_manager.hpp"

int main()
{
    shared_memory::clear_shared_memory(
        dynamic_graph_manager::DynamicGraphManager::shared_memory_name_);
}
