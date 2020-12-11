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

#include "shared_memory/locked_condition_variable.hpp"
#include <real_time_tools/thread.hpp>

#include <dynamic_graph_manager/tools.hpp>

namespace dynamic_graph_manager
{

class DGMHead
{
public:
    DGMHead(std::string& yaml_file);

    Eigen::Ref<Eigen::VectorXd> get_sensor(std::string& name);

    void set_control(std::string& name, Eigen::Ref<Eigen::VectorXd> vector);

    void lock_conditional_variable();
    void unlock_conditional_variable();

    void read();

    void write();

    void notify_all();

    void wait();

    void processing_data();

    void end_processing_data();

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

    bool is_alive_;
};

}
