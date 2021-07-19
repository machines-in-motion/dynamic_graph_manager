/**
 * @file dgm_head.hpp
 * @author Julian Viereck <jviereck@tuebingen.mpg.de>
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-07
 */

#pragma once

#include <Eigen/Dense>

#include <real_time_tools/thread.hpp>
#include "shared_memory/locked_condition_variable.hpp"

#include <dynamic_graph_manager/tools.hpp>

namespace dynamic_graph_manager
{
class DGMHead
{
public:
    /**
     * @brief Given a DGM yaml file, creates a DGMHead to interact via shared
     * memory with the dynamic graph manager hardware process.
     */
    DGMHead(std::string& yaml_file);

    /**
     * @brief Given the name, returns the sensor value by reference.
     */
    Eigen::Ref<Eigen::VectorXd> get_sensor(std::string& name);

    /**
     * @brief Sets the control value for a given name.
     */
    void set_control(std::string& name, Eigen::Ref<Eigen::VectorXd> vector);

    /**
     * @brief Locks the scope of the conditional variable used to synchronize
     * the DGM hardware and control process.
     */
    void lock_conditional_variable();

    /**
     * @brief Unlocks the scope of the conditional variable used to synchronize
     * the DGM hardware and control process.
     */
    void unlock_conditional_variable();

    /**
     * @brief Reads the current sensor values from shared memory. The values
     * returned from `get_sensor` are updated after this call.
     */
    void read();

    /**
     * @brief Writes the control updates made via calls to `set_control` to the
     * shared memory.
     */
    void write();

    /**
     * @brief Calls `notify_all` on the shared conditional variable used to
     * synchronize the hardware and control process.
     */
    void notify_all();

    /**
     * @brief Waits until the conditional variable used to synchronize the
     * hardware and control process becomes available and locks it. Used to
     * synchronize the hardware and control process.
     */
    void wait();

    /**
     * @brief Implementation of the real time processing thread.
     */
    void processing_data();

    /**
     * @brief Notifies the real time processing thread to stop processing the
     * data.
     */
    void end_processing_data();

    /**
     * @brief Starts the real time processing thread to read/write data from
     * shared memory.
     */
    void start_realtime_processing_thread();

protected:
    static THREAD_FUNCTION_RETURN_TYPE processing_data(void* instance_pointer)
    {
        ((DGMHead*)(instance_pointer))->processing_data();
        return THREAD_FUNCTION_RETURN_VALUE;
    }

    /**
     * @brief sensors_map_ is a map of dynamicgraph::Vector. They represent
     * all the sensors data measured on the robot.
     */
    VectorDGMap sensors_map_;

    /**
     * @brief motor_controls_map_ is a map of dynamicgraph::Vector. They
     * represent all the controls to be sent to the robot.
     */
    VectorDGMap motor_controls_map_;

    /**
     * @brief cond_var_sensors_ this condition variable allow the computation of
     * the dynamic graph just after the acquisition of the sensors
     */
    std::unique_ptr<shared_memory::LockedConditionVariable> cond_var_;

    /**
     * @brief Thread used to read and write the data.
     */
    real_time_tools::RealTimeThread processing_thread_;

    /**
     * @brief If the processing thread should be alive or not.
     */
    bool is_alive_;

    /**
     * @brief Name of the shared memory.
     */
    std::string shared_memory_name_;

    /**
     * @brief Name of the cond variable in shared memory.
     */
    std::string cond_var_name_;
};

}  // namespace dynamic_graph_manager
