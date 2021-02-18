/**
 * @file ros_init.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include "dynamic_graph_manager/ros.hpp"

#include <atomic>
#include <fstream>

#include "real_time_tools/timer.hpp"

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
 * After rclcpp::init being called we create the node handle which allows in
 * turn to advertize the ROS services, or create topics (data pipe).
 *
 */
static GlobalListOfRosNodeType GLOBAL_LIST_OF_ROS_NODE;

/**
 * @brief Small class allowing to start a ROS spin in a thread.
 */
class Executor
{
public:
    Executor() : ros_executor_(rclcpp::executor::ExecutorArgs(), 4)
    {
        is_thread_running_ = false;
        is_spinning_ = false;
        list_node_added_.clear();
    }

    /**
     * @brief Upon destruction close the thread and stop spinning.
     *
     * @return
     */
    ~Executor()
    {
        stop_spinning();
    }

    /**
     * @brief Start ros_spin in a different thread to not block the current one.
     */
    void spin_non_blocking()
    {
        if (!is_thread_running_ && !is_spinning_)
        {
            std::cout << "Start ros spin in thread." << std::endl;
            // Marking thread as started to avoid a second thread from getting started.
            is_thread_running_ = true;
            thread_ = std::thread(&Executor::thread_callback, this);
        }
    }

    /**
     * @brief Block the current thread and make ROS spinning.
     */
    void spin()
    {
        if(is_thread_running_)
        {
            while(ros_ok() && is_thread_running_)
            {
                real_time_tools::Timer::sleep_sec(0.1);
            }
        }else{
            is_spinning_ = true;
            ros_executor_.spin();
            is_spinning_ = false;
        }
    }

    void add_node(const std::string& ros_node_name)
    {
        if (std::find(list_node_added_.begin(),
                      list_node_added_.end(),
                      ros_node_name) == list_node_added_.end())
        {
            list_node_added_.push_back(ros_node_name);
            ros_executor_.add_node(get_ros_node(ros_node_name));
        }
    }

    void remove_node(const std::string& ros_node_name)
    {
        std::deque<std::string>::iterator el = std::find(
            list_node_added_.begin(), list_node_added_.end(), ros_node_name);
        if (el != list_node_added_.end())
        {
            list_node_added_.erase(el);
            assert(std::find(list_node_added_.begin(),
                             list_node_added_.end(),
                             ros_node_name) == list_node_added_.end() &&
                   "The node has not been removed properly.");
            ros_executor_.remove_node(get_ros_node(ros_node_name));
        }
    }

    /**
     * @brief Stop the spinning al together. Callable in a different thread.
     */
    void stop_spinning()
    {
        while(is_thread_running_ || is_spinning_)
        {
            ros_executor_.cancel();
            real_time_tools::Timer::sleep_sec(0.1);
        }
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

private:
    /**
     * @brief Thread callback function
     */
    void thread_callback()
    {
        ros_executor_.spin();
        is_thread_running_ = false;
    }

    /**
     * @brief Check if the thread is running.
     */
    std::atomic<bool> is_thread_running_;

    /**
     * @brief Check if the the executor is spinning.
     */
    std::atomic<bool> is_spinning_;

    /**
     * @brief Thread in which the EXECUTOR spins.
     */
    std::thread thread_;

    /**
     * @brief Object that execute the ROS callbacks in a different thread.
     */
    RosExecutor ros_executor_;

    std::deque<std::string> list_node_added_;

};  // class Executor

typedef std::shared_ptr<Executor> ExecutorPtr;

/**
 * @brief EXECUTOR is a ros object that handles in a global way
 * all the ros callbacks and interruption. Call EXECUTOR.spin()
 * in order to start handling the callback in a separate thread.
 */
ExecutorPtr EXECUTOR = nullptr;

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

ExecutorPtr get_ros_executor()
{
    ros_init();
    if (!EXECUTOR)
    {
        EXECUTOR = std::make_shared<Executor>();
    }
    return EXECUTOR;
}

RosNodePtr get_ros_node(std::string node_name)
{
    ros_init();
    if (!ros_node_exists(node_name))
    {
        GLOBAL_LIST_OF_ROS_NODE[node_name] = RosNodePtr(nullptr);
    }
    if (!GLOBAL_LIST_OF_ROS_NODE[node_name] ||
        GLOBAL_LIST_OF_ROS_NODE[node_name].get() == nullptr)
    {
        /** RosNode instanciation */
        GLOBAL_LIST_OF_ROS_NODE[node_name] =
            std::make_shared<RosNode>(node_name, "dynamic_graph_manager");
    }
    /** Return a reference to the node handle so any function can use it */
    return GLOBAL_LIST_OF_ROS_NODE[node_name];
}

void ros_add_node_to_executor(const std::string& node_name)
{
    get_ros_executor()->add_node(node_name);
}

void ros_shutdown(std::string node_name)
{
    if (GLOBAL_LIST_OF_ROS_NODE.find(node_name) ==
        GLOBAL_LIST_OF_ROS_NODE.end())
    {
        return;
    }
    get_ros_executor()->remove_node(node_name);
    GLOBAL_LIST_OF_ROS_NODE.erase(node_name);
}

void ros_shutdown()
{
    // rclcpp::shutdown();
}

void ros_clean()
{
    ros_stop_spinning();
    GlobalListOfRosNodeType::iterator ros_node_it =
        GLOBAL_LIST_OF_ROS_NODE.begin();
    while(!GLOBAL_LIST_OF_ROS_NODE.empty())
    {
        ros_shutdown(ros_node_it->first);
        ros_node_it = GLOBAL_LIST_OF_ROS_NODE.begin();
    }
    GLOBAL_LIST_OF_ROS_NODE.clear();
}

bool ros_ok()
{
    return rclcpp::ok() && rclcpp::is_initialized();
}

void ros_spin()
{
    get_ros_executor()->spin();
}

void ros_spin_non_blocking()
{
    get_ros_executor()->spin_non_blocking();
}

void ros_stop_spinning()
{
    get_ros_executor()->stop_spinning();
}

}  // end of namespace dynamic_graph_manager.
