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
 * @brief Small class allowing to start a ROS spin in a thread.
 */
class RosExecutorThread
{
public:
    RosExecutorThread() : ros_executor_thread_(nullptr)
    {
    }

    /**
     * @brief Start ros_spin in a different thread.
     */
    void spin()
    {
        if (!ros_executor_thread_)
        {
            ros_executor_thread_ = std::make_shared<std::thread>(&ros_spin);
        }
    }

    ~RosExecutorThread()
    {
        if (ros_executor_thread_)
        {
            if (ros_executor_thread_->joinable())
            {
                ros_executor_thread_->join();
            }
        }
    }

private:
    /**
     * @brief ROS_EXECUTOR_THREAD is a thread in which the ROS_EXECUTOR spins.
     */
    std::shared_ptr<std::thread> ros_executor_thread_;
};

/**
 * @brief ROS_EXECUTOR is a ros object that handles in a global way
 * all the ros callbacks and interruption. Call ROS_EXECUTOR.spin()
 * in order to start handling the callback in a separate thread.
 */
std::shared_ptr<RosExecutor> ROS_EXECUTOR = nullptr;

/**
 * @brief Object that execute the ROS callbacks in a different thread.
 */
std::shared_ptr<RosExecutorThread> ROS_EXECUTOR_THREAD = nullptr;

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

/*
 * Public Functions => user API, see ros.hpp for the docstrings.
 */

bool ros_node_exists(std::string node_name)
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

RosExecutorPtr get_ros_executor()
{
    ros_init();
    if (ROS_EXECUTOR == nullptr)
    {
        ROS_EXECUTOR =
            std::make_shared<RosExecutor>(rclcpp::executor::ExecutorArgs(), 4);
    }
    return ROS_EXECUTOR;
}

RosNodePtr get_ros_node(std::string node_name)
{
    ros_init();
    if (!ros_node_exists(node_name))
    {
        GLOBAL_LIST_OF_ROS_NODE[node_name] = RosNodePtr(nullptr);
    }
    if (GLOBAL_LIST_OF_ROS_NODE[node_name] == nullptr)
    {
        /** RosNode instanciation */
        GLOBAL_LIST_OF_ROS_NODE[node_name] =
            std::make_shared<RosNode>(node_name, "dynamic_graph_manager");

        RosExecutorPtr executor = get_ros_executor();
        executor->add_node(GLOBAL_LIST_OF_ROS_NODE[node_name]);
    }
    /** Return a reference to the node handle so any function can use it */
    return GLOBAL_LIST_OF_ROS_NODE[node_name];
}

void ros_spin()
{
    RosExecutorPtr executor = get_ros_executor();
    executor->spin();
}

void ros_shutdown(std::string node_name)
{
    if (GLOBAL_LIST_OF_ROS_NODE.find(node_name) ==
        GLOBAL_LIST_OF_ROS_NODE.end())
    {
        return;
    }
    GLOBAL_LIST_OF_ROS_NODE.erase(node_name);
}

void ros_shutdown()
{
    rclcpp::shutdown();
}

bool ros_ok()
{
    return rclcpp::ok() && rclcpp::is_initialized();
}

void ros_spin_non_blocking()
{
    if (!ROS_EXECUTOR_THREAD)
    {
        ROS_EXECUTOR_THREAD = std::make_shared<RosExecutorThread>();
    }
    ROS_EXECUTOR_THREAD->spin();
}

}  // end of namespace dynamic_graph_manager.
