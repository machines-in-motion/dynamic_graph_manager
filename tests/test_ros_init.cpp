/**
 * @file test_get_ros_node.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include "test_common.hpp"

/***************************
 * SETUP THE TESTING CLASS *
 ***************************/

using namespace std::chrono_literals;
using namespace dynamic_graph_manager;

/**
 * @brief This is the test environment
 */
class TestRosInit : public ::testing::Test
{
    /**
     * @brief SetUp, is executed before the unit tests
     */
    void SetUp()
    {
        ros_spin_non_blocking();
    }

    /**
     * @brief TearDown, is executed after the unit tests
     */
    void TearDown()
    {
        ros_clean();
    }
};

/**
 * @brief This is the test environment
 */
class TestRosInitGlobalVar : public ::testing::Test
{
};

/*****************
 * Start Testing *
 *****************/

/**
 * Here we check that we can create a ros node
 */
TEST_F(TestRosInitGlobalVar, test_create_node)
{
    /* Setup */
    std::string node_name = "my_global_node";
    get_ros_node(node_name);

    /* Test */
    ASSERT_TRUE(ros_node_exists(node_name));
}

/**
 * Here we check that we still have the ros node created (global variable)
 */
TEST_F(TestRosInitGlobalVar, test_global_variable_deleted_after_tear_down)
{
    /* Setup */
    std::string node_name = "my_global_node";

    /* Test */
    ASSERT_TRUE(ros_node_exists(node_name));
}

/**
 * Here we check that the destruction of the variable
 */
TEST_F(TestRosInitGlobalVar, test_delete_global_variable)
{
    /* Setup */
    std::string node_name = "my_global_node";
    ros_shutdown(node_name);

    /* Test */
    ASSERT_FALSE(ros_node_exists(node_name));
}

/**
 * Here we check if we can destruct the object several time
 */
TEST_F(TestRosInit, test_multiple_shutdown_call)
{
    /* Setup */
    std::string node_name = "my_global_node";

    /* Test */
    ASSERT_NO_THROW(for (unsigned i = 0; i < 20; ++i) {
        ros_shutdown(node_name);
        real_time_tools::Timer::sleep_sec(0.001);
    });
}

/**
 * @brief Dummy function to be called as ros services
 *
 * @return true
 */
bool simple_service(std_srvs::srv::Empty::Request::SharedPtr,
                    std_srvs::srv::Empty::Response::SharedPtr)
{
    return true;
}

/**
 * Check if services are available
 */
TEST_F(TestRosInit, test_services_available)
{
    /* Setup */
    // create a node and a service
    RosNodePtr n0 = get_ros_node("node_0");
    ros_add_node_to_executor("node_0");
    std::string service_name = "/simple_service";
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server =
        n0->create_service<std_srvs::srv::Empty>(service_name, &simple_service);

    /* Test */
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
        n0->create_client<std_srvs::srv::Empty>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));
}

/**
 * Check if services are not longer available after shutdown
 */
TEST_F(TestRosInit, test_services_shut_down)
{
    /* Setup */
    std::string service_name = "/simple_service";

    // create a node and a service and shutdown the node
    {
        RosNodePtr n0 = get_ros_node("node_0");
        ros_add_node_to_executor("node_0");
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server =
            n0->create_service<std_srvs::srv::Empty>(service_name,
                                                     &simple_service);
        ros_shutdown("node_0");
    }

    // create a client to this service
    RosNodePtr n1 = get_ros_node("node_1");
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
        n1->create_client<std_srvs::srv::Empty>(service_name);

    /* Test */
    ASSERT_FALSE(client->wait_for_service(100ms));
}

/**
 * Check if shutdown without name kills all created nodes.
 */
TEST_F(TestRosInit, test_killall_nodes)
{
    /* Setup */
    // creates tones of nodes
    unsigned int nb_nodes = 20;
    for (unsigned int i = 0; i < nb_nodes; ++i)
    {
        {
            std::ostringstream os;
            os << "node_" << i;
            get_ros_node(os.str());
        }
    }

    // kill them all
    ros_clean();

    /* Test if they are all killed */
    for (unsigned int i = 0; i < nb_nodes; ++i)
    {
        {
            std::ostringstream os;
            os << "node_" << i;
            ASSERT_FALSE(ros_node_exists(os.str()));
        }
    }
}
