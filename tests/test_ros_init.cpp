/**
 * @file test_ros_init.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <gtest/gtest.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include "dynamic_graph_manager/ros.hpp"

/***************************
 * SETUP THE TESTING CLASS *
 ***************************/

using namespace dynamic_graph_manager;

/**
 * @brief This is the test environment
 */
class TestRosInit : public ::testing::Test
{
};

/*****************
 * Start Testing *
 *****************/

/**
 * Here we check that we can create a ros node
 */
TEST_F(TestRosInit, test_create_node)
{
    // setup
    std::string node_name = "my_global_node";
    ros_init(node_name);

    // test
    ASSERT_TRUE(ros_exist(node_name));
}

/**
 * Here we check that we still have the ros node created (global variable)
 */
TEST_F(TestRosInit, test_global_variable)
{
    // setup
    std::string node_name = "my_global_node";

    // test
    ASSERT_TRUE(ros_exist(node_name));
}

/**
 * Here we check that the destruction of the variable
 */
TEST_F(TestRosInit, test_delete_global_variable)
{
    // setup
    std::string node_name = "my_global_node";
    ros_shutdown(node_name);

    // test
    ASSERT_FALSE(ros_exist(node_name));
}

/**
 * Here we check that we have a spinner running
 */
TEST_F(TestRosInit, test_spinner)
{
    // setup
    std::string node_name = "my_global_node";
    ros::AsyncSpinner& spinner = ros_spinner(node_name);

    // test
    ASSERT_TRUE(&spinner != nullptr);
}

/**
 * Here we check if we can destruct the object several time
 */
TEST_F(TestRosInit, test_multiple_shutdown_call)
{
    // setup
    std::string node_name = "my_global_node";

    // test
    ASSERT_NO_THROW(
        for (unsigned i = 0; i < 200; ++i) { ros_shutdown(node_name); });
}

/**
 * @brief Dummy function to be called as ros services
 *
 * @return true
 */
bool simple_service(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    return true;
}

/**
 * Check if services are available
 */
TEST_F(TestRosInit, test_services_available)
{
    // setup
    // create a node and a service
    ros::NodeHandle& n0 = ros_init("node_0");
    std::string service_name = "/simple_service";
    ros::ServiceServer server =
        n0.advertiseService(service_name, &simple_service);
    std_srvs::Empty srv;

    // test
    ros::ServiceClient client = n0.serviceClient<std_srvs::Empty>(service_name);
    ASSERT_TRUE(client.waitForExistence());
    ASSERT_TRUE(ros::service::exists(service_name, true));

    // tear down
    ros_shutdown("node_0");
}

/**
 * Check if services are not longer available after shutdown
 */
TEST_F(TestRosInit, test_services_shut_down)
{
    // setup
    // create a node and a service and shutdown the node
    ros::NodeHandle& n0 = ros_init("node_0");
    n0.advertiseService("simple_service", &simple_service);
    ros_shutdown("node_0");

    // create a client to this service
    ros::NodeHandle& n1 = ros_init("node_1");
    std_srvs::Empty srv;
    ros::ServiceClient simple_service_client =
        n1.serviceClient<std_srvs::Empty>("/node_0/simple_service");

    // test
    ASSERT_FALSE(simple_service_client.waitForExistence(ros::Duration(0.0009)));
    ASSERT_FALSE(simple_service_client.call(srv));

    // tear down
    ros_shutdown("node_1");
}

/**
 * Check if shutdown without name kills all created nodes.
 */
TEST_F(TestRosInit, test_killall_nodes)
{
    // setup
    // creates tones of nodes
    unsigned int nb_nodes = 100;
    for (unsigned int i = 0; i < nb_nodes; ++i)
    {
        {
            std::ostringstream os;
            os << "node_" << i;
            ros_init(os.str());
        }
    }

    // kill them all
    ros_shutdown();

    // test if they are all killed
    for (unsigned int i = 0; i < nb_nodes; ++i)
    {
        {
            std::ostringstream os;
            os << "node_" << i;
            ASSERT_FALSE(ros_exist(os.str()));
        }
    }
}
