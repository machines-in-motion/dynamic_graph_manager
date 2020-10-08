/**
 * @file ros_init.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <dynamic_graph_manager/ros.hpp>
#include <fstream>

namespace dynamic_graph_manager
{
/*
 * Private methods
 */

/**
 * @brief Shortcut.
 */
typedef std::map<std::string, RosNodePtr> GlobalListOfRosNodeType;

/**
 * @brief GLOBAL_LIST_OF_ROS_NODE is global variable that acts as a singleton on
 * the ROS node handle and the spinner.
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
static GlobalListOfRosNodeType GLOBAL_LIST_OF_ROS_NODE;

/**
 * @brief ros_executor_ is a ros object that handles in a global way
 * all the ros callbacks and interruption. Call ros_executor_.start()
 * in order to start handling the callback in a separate thread.
 */
std::shared_ptr<RosExecutor> ros_executor_ = nullptr;

/**
 * @brief Private function that allow us to get the current executable name.
 *
 * @return std::string the current executable name.
 */
std::string executable_name()
{
#if defined(PLATFORM_POSIX) || \
    defined(__linux__)  // check defines for your setup

    std::string sp;
    std::ifstream("/proc/self/comm") >> sp;
    return sp;

#elif defined(_WIN32)

    char buf[MAX_PATH];
    GetModuleFileNameA(nullptr, buf, MAX_PATH);
    return buf;

#else

    static_assert(false, "unrecognized platform");

#endif
}

/**
 * @brief Private function taht checks if the Node has been created.
 *
 * @param node_name
 * @return true
 * @return false
 */
bool ros_exist(std::string node_name)
{
    if (GLOBAL_LIST_OF_ROS_NODE.find(node_name) ==
        GLOBAL_LIST_OF_ROS_NODE.end())
    {
        return false;
    }
    if (GLOBAL_LIST_OF_ROS_NODE.at(node_name) == nullptr)
    {
        return false;
    }
    return true;
}

/**
 * @brief Private function that allow us to initialize ROS only once.
 */
void ros_init()
{
    if (!rclcpp::is_initialized())
    {
        /** call rclcpp::init */
        int argc = 1;
        char* arg0 = strdup(executable_name().c_str());
        char* argv[] = {arg0, nullptr};
        rclcpp::init(argc, argv);
        free(arg0);
    }
}

/**
 * @brief Get ros executor global variable in order to execute the ROS nodes.
 *
 * @return RosExecutorPtr
 */
RosExecutorPtr get_ros_executor()
{
    ros_init();
    if (ros_executor_ == nullptr)
    {
        ros_executor_ = std::make_shared<RosExecutor>();
    }
    return ros_executor_;
}

/*
 * Public Functions => user API, see ros.hpp for the docstrings.
 */

RosNodePtr get_ros_node(std::string node_name)
{
    ros_init();
    if (!ros_exist(node_name))
    {
        GLOBAL_LIST_OF_ROS_NODE[node_name] = RosNodePtr(nullptr);
    }
    if (GLOBAL_LIST_OF_ROS_NODE[node_name] == nullptr)
    {
        /** RosNode instanciation */
        GLOBAL_LIST_OF_ROS_NODE[node_name] =
            std::make_shared<RosNode>(node_name, "dynamic_graph_manager");
    }
    /** Return a reference to the node handle so any function can use it */
    return GLOBAL_LIST_OF_ROS_NODE[node_name];
}

void ros_spin()
{
    RosExecutorPtr executor = get_ros_executor();
    for (GlobalListOfRosNodeType::iterator it = GLOBAL_LIST_OF_ROS_NODE.begin();
         it != GLOBAL_LIST_OF_ROS_NODE.end();
         ++it)
    {
        executor->add_node(it->second);
    }    
    executor->spin();
}

void ros_shutdown()
{
    rclcpp::shutdown();
}

bool ros_ok()
{
    return rclcpp::ok() && rclcpp::is_initialized();
}

}  // end of namespace dynamic_graph_manager.
