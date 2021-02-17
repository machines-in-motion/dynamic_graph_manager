/**
 * @file
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 *
 * @brief Purge the shared memory used by the dynamic_graph_manager
 */

#include "shared_memory/shared_memory.hpp"

int main()
{
    shared_memory::clear_shared_memory("dgm_shm_name");
    shared_memory::clear_shared_memory("DGM_ShM");
}
