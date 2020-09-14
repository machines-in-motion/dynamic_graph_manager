/**
 * @file ros_init.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <dynamic_graph_manager/ros.hpp>

namespace dynamic_graph_manager
{
/**
 * @brief GLOBAL_ROS_VAR is global variable that acts as a singleton on the
 * ROS node handle and the spinner.
 *
 * The use of the std::unique_ptr allows to delete the object and re-create
 * at will. It is usefull to reset the ros environment specifically for
 * unittesting.
 *
 * If the node handle does not exist we call the global method rclcpp::init.
 * This method has for purpose to initialize the ROS environment. The
 * creation of ROS object is permitted only after the call of this function.
 * After rclcpp::init being called we create the node hanlde which allows in
 * turn to advertize the ROS services, or create topics (data pipe).
 *
 */
static std::map<std::string, std::unique_ptr<GlobalRos> > GLOBAL_ROS_VAR;

RosNodePtr get_ros_node(std::string node_name)
{
    if (!rclcpp::is_initialized())
    {
        /** call rclcpp::init */
        int argc = 1;
        char* arg0 = strdup(node_name.c_str());
        char* argv[] = {arg0, nullptr};
        rclcpp::init(argc, argv);
        free(arg0);
    }
    if (!ros_exist(node_name))
    {
        GLOBAL_ROS_VAR[node_name] = std::make_unique<GlobalRos>();
    }
    if (GLOBAL_ROS_VAR[node_name]->node_ == nullptr)
    {
        /** RosNode instanciation */
        GLOBAL_ROS_VAR[node_name]->node_ =
            std::make_shared<RosNode>(node_name);
    }
    /**
     * If spinner is not created we create it. Here we can safely assume that
     * rclcpp::init was called before.
     */
    if (GLOBAL_ROS_VAR[node_name]->executor_ == nullptr)
    {
        /** create the spinner */
        GLOBAL_ROS_VAR[node_name]->executor_ = std::make_shared<RosExecutor>();
    }
    /** Return a reference to the node handle so any function can use it */
    return GLOBAL_ROS_VAR[node_name]->node_;
}

RosExecutorPtr get_ros_executor(std::string node_name)
{
    if (!ros_exist(node_name))
    {
        dynamic_graph_manager::get_ros_node(node_name);
    }
    assert(GLOBAL_ROS_VAR[node_name]->executor_ != nullptr &&
           "The spinner must have been created by now.");
    return GLOBAL_ROS_VAR[node_name]->executor_;
}

void ros_spin(std::string node_name)
{
    RosExecutorPtr executor = get_ros_executor(node_name);
    executor->add_node(get_ros_node(node_name));
    executor->spin();
}

void ros_shutdown(std::string node_name)
{
    GLOBAL_ROS_VAR.erase(node_name);
}

void ros_shutdown()
{
    for (std::map<std::string, std::unique_ptr<GlobalRos> >::iterator it =
             GLOBAL_ROS_VAR.begin();
         it != GLOBAL_ROS_VAR.end();
         ++it)
    {
        it->second.reset(nullptr);
    }
    GLOBAL_ROS_VAR.clear();
}

bool ros_exist(std::string node_name)
{
    if (GLOBAL_ROS_VAR.find(node_name) == GLOBAL_ROS_VAR.end())
    {
        return false;
    }
    if (GLOBAL_ROS_VAR.at(node_name) == nullptr)
    {
        return false;
    }
    return true;
}

bool ros_ok()
{
    return rclcpp::ok() && rclcpp::is_initialized();
}

}  // end of namespace dynamic_graph_manager.
