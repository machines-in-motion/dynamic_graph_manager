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

#include <dynamic_graph_manager/tools.hpp>

namespace dynamic_graph_manager
{

class DGMHead
{
    DGMHead(std::string& yaml_file);

    Eigen::Ref<Eigen::VectorXd> get_sensor(std::string& name);

    void set_control(std::string& name, Eigen::Ref<Eigen::VectorXd> vector);

    void read();

    void put_notify_wait();

protected:
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
}

}
