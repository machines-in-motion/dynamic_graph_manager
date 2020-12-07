/**
 * @file dgm_head.cpp
 * @author Julian Viereck <jviereck@tuebingen.mpg.de>
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-07
 */

#include <dynamic_graph_manager/dgm_head.hpp>

// get the yaml configuration
#include "yaml_utils/yaml_cpp_fwd.hpp"

namespace dynamic_graph_manager
{

const std::string sensors_map_name_ = "sensors_map";
const std::string motor_controls_map_name_ = "motor_controls_map";
const std::string shared_memory_name_ = "DGM_ShM";
const std::string cond_var_name_ = "cond_var";

DGMHead::DGMHead(std::string& yaml_file)
{
    // we build the condition variables after the fork (seems safer this way)
    cond_var_ = std::make_unique<shared_memory::LockedConditionVariable>(
        cond_var_name_, false);

    std::cout << "Loading parameters from " << yaml_file << std::endl;
    YAML::Node param = YAML::LoadFile(yaml_params_file);
    parse_yaml_node(param["device"], sensors_map_, motor_controls_map_);

    cond_var_->lock_scope();
}

void DGMHead::read()
{
    // Get the sensor values from shared memory.
    shared_memory::get(shared_memory_name_, sensors_map_name_, sensors_map_);
}

void DGMHead::put_notify_wait()
{
    // Write the command to the shared memory.
    shared_memory::set(
        shared_memory_name_, motor_controls_map_name_, motor_controls_map_);

    // Notify the hardware_communication process that the control has been
    // done.
    cond_var_->notify_all();

    // Wait that the hardware_communication process acquiers the data.
    cond_var_->wait();
}

Eigen::Ref<Eigen::VectorXd> DGMHead::get_sensor(std::string& name)
{
    if (sensors_map_.count(name) == 0)
    {
        throw std::runtime_error('Unknown sensor name: ' + name);
    }
    return sensors_map_[name];
}

void DGMHead::set_control(std::string& name, Eigen::Ref<Eigen::VectorXd> vector)
{
    if (motor_controls_map_.count(name) == 0)
    {
        throw std::runtime_error('Unknown control name: ' + name);
    }
    motor_controls_map_[name] = vector;
}

} // namespace dynamic_graph_manager
