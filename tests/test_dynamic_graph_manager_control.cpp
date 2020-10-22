/**
 * @file test_dynamic_graph_manager.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <gtest/gtest.h>
#include <signal.h>
#include <sys/types.h>

#include <chrono>
#include <thread>

// We use another class of this package which has an independant unittest.
#include "dynamic_graph_manager/ros.hpp"

// The class to test
#include "dynamic_graph_manager/dynamic_graph_manager.hpp"

// Service for the user command unittest
#include "dynamic_graph_manager/srv/test_user_cmd_bool.hpp"

// SimpleDGM class, the header is located in the demos/ folder of this repos.
#include "simple_dgm.hpp"

using namespace std::chrono_literals;
using namespace dynamic_graph_manager;

/**
 * SETUP THE TESTING CLASS
 */

/**
 * @brief The DISABLED_TestDynamicGraphManager class is used to disable test
 */
class DISABLED_TestDynamicGraphManager : public ::testing::Test
{
public:
    YAML::Node params_;
};

const std::string ROS_NODE_NAME = "test_dynamic_graph_manager_control";

/**
 * @brief The TestDynamicGraphManager class: test suit template for setting up
 * the unit tests for the DynamicGraphManager
 */
class TestDynamicGraphManager : public ::testing::Test
{
protected:
    /**
     * @brief SetUp, is executed before the unit tests
     */
    void SetUp()
    {
        params_ =
            YAML::LoadFile(TEST_CONFIG_PATH + std::string("simple_robot.yaml"));
        shared_memory::clear_shared_memory(
            DynamicGraphManager::shared_memory_name_);
        ros_spin_non_blocking();
    }

    /**
     * @brief TearDown, is executed after the unit tests
     */
    void TearDown()
    {
        ros_shutdown();
    }

    /**
     * @brief Params of the DGM
     */
    YAML::Node params_;
};

void start_dg_ros_service()
{
    // service name
    std::string service_name = "/dynamic_graph_manager/start_dynamic_graph";
    // Create a client from a temporary node
    RosNodePtr ros_node = get_ros_node(ROS_NODE_NAME);
    auto client = ros_node->create_client<std_srvs::srv::Empty>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));

    // fill the command message
    std_srvs::srv::Empty::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Empty::Request>();
    std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> response;
    // call the service
    response = client->async_send_request(request);
    // wait for the answer
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
}

void stop_dg_ros_service()
{
    // service name
    std::string service_name = "/dynamic_graph_manager/stop_dynamic_graph";
    // Create a client from a temporary node
    RosNodePtr ros_node = get_ros_node(ROS_NODE_NAME);
    auto client = ros_node->create_client<std_srvs::srv::Empty>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));

    // fill the command message
    std_srvs::srv::Empty::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Empty::Request>();
    std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> response;
    // call the service
    response = client->async_send_request(request);
    // wait for the answer
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
}

void start_run_python_command_ros_service(std::string cmd)
{
    // service name
    std::string service_name = "/dynamic_graph_manager/run_python_command";
    // Create a client from a temporary node
    RosNodePtr ros_node = get_ros_node(ROS_NODE_NAME);
    auto client = ros_node->create_client<srv::RunPythonCommand>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));

    // fill the command message
    srv::RunPythonCommand::Request::SharedPtr request =
        std::make_shared<srv::RunPythonCommand::Request>();
    std::shared_future<srv::RunPythonCommand::Response::SharedPtr> response;
    request->input = cmd;
    // call the service
    response = client->async_send_request(request);
    // wait for the answer
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
}

void start_run_python_script_ros_service(std::string file_name)
{
    // service name
    std::string service_name = "/dynamic_graph_manager/run_python_script";
    // Create a client from a temporary node
    RosNodePtr ros_node = get_ros_node(ROS_NODE_NAME);
    auto client = ros_node->create_client<srv::RunPythonFile>(service_name);
    ASSERT_TRUE(client->wait_for_service(1s));

    // fill the command message
    srv::RunPythonFile::Request::SharedPtr request =
        std::make_shared<srv::RunPythonFile::Request>();
    std::shared_future<srv::RunPythonFile::Response::SharedPtr> response;
    request->input = file_name;
    // call the service
    response = client->async_send_request(request);
    // wait for the answer
    ASSERT_TRUE(rclcpp::spin_until_future_complete(ros_node, response) ==
                rclcpp::executor::FutureReturnCode::SUCCESS);
}

/*****************
 * Start Testing *
 *****************/

/**
 * Here we check that the constructor throw no error
 */
TEST_F(TestDynamicGraphManager, test_constructor)
{
    /** Test */
    ASSERT_NO_THROW(DynamicGraphManager dgm;);
}

/**
 * Here we check that the destructor throw no error
 */
TEST_F(TestDynamicGraphManager, test_destructor)
{
    /** Test */
    ASSERT_NO_THROW({ DynamicGraphManager dgm; });
}

/**
 * Here we check that the initialization throw no error
 */
TEST_F(TestDynamicGraphManager, test_initialize)
{
    /** Test */
    ASSERT_NO_THROW({
        DynamicGraphManager dgm;
        dgm.initialize(params_);
    });
}

/**
 * Here we check that the initialization does not create nor initialize a ros
 * node.
 */
TEST_F(TestDynamicGraphManager, test_initialize_no_dg_node_handle)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);

    /** Test */
    ASSERT_FALSE(ros_node_exists(dgm.dg_ros_node_name_));
}

/**
 * Here we check that the initialization does not create nor initialize a ros
 * node.
 */
TEST_F(TestDynamicGraphManager, test_initialize_no_hwc_node_handle)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);

    /** Test */
    ASSERT_FALSE(ros_node_exists(dgm.hw_com_ros_node_name_));
}

/**
 * Here we check that the initialization of the dynamic graph process creates
 * the ros node properly.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_node_handle_exists)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();

    /** Test */
    ASSERT_TRUE(ros_node_exists(dgm.dg_ros_node_name_));
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. The "/dynamic_graph/start_dynamic_graph" ros sservice must be
 * available by now.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_start_dg_exist)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();
    RosNodePtr ros_node = get_ros_node(dgm.dg_ros_node_name_);
    auto client = ros_node->create_client<std_srvs::srv::Empty>(
        "/dynamic_graph_manager/start_dynamic_graph");

    /** Test */
    ASSERT_TRUE(client->wait_for_service(1s));
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. The "/dynamic_graph/stop_dynamic_graph" ros sservice must be
 * available by now.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_stop_dg_exist)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();
    RosNodePtr ros_node = get_ros_node(dgm.dg_ros_node_name_);
    auto client = ros_node->create_client<std_srvs::srv::Empty>(
        "/dynamic_graph_manager/stop_dynamic_graph");

    /** Test */
    ASSERT_TRUE(client->wait_for_service(1s));
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. The "/dynamic_graph/run_python_command" ros sservice must be
 * available by now.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_run_python_cmd_exist)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();
    RosNodePtr ros_node = get_ros_node(dgm.dg_ros_node_name_);
    auto client = ros_node->create_client<srv::RunPythonCommand>(
        "/dynamic_graph_manager/run_python_command");

    /** Test */
    ASSERT_TRUE(client->wait_for_service(1s));
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. The "/dynamic_graph/run_python_script" ros sservice must be
 * available by now.
 */
TEST_F(TestDynamicGraphManager, test_init_dg_process_run_python_file_exist)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();
    RosNodePtr ros_node = get_ros_node(dgm.dg_ros_node_name_);
    auto client = ros_node->create_client<srv::RunPythonFile>(
        "/dynamic_graph_manager/run_python_file");

    /** Test */
    ASSERT_TRUE(client->wait_for_service(1s));
}

/**
 * Here we check that the initialization of the hardware communication fails
 * if we call the methode from the raw dynamic graph manager class.
 */
TEST_F(TestDynamicGraphManager, test_initialize_hwc_throw)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);

    /** Test */
    ASSERT_THROW(dgm.initialize_hardware_communication_process(),
                 std::runtime_error);
}

/**
 * Here we check that the initialization of the hardware communication runs
 * smoothly using a daughter class of the dynamic graph manager.
 */
TEST_F(TestDynamicGraphManager, test_initialize_hwc_no_throw)
{
    /** Setup */
    SimpleDGM dgm;
    dgm.initialize(params_);

    /** Test */
    ASSERT_NO_THROW(dgm.initialize_hardware_communication_process(););
}

/**
 * Here we check that the initialization of the hardware communication runs
 * smoothly using a daughter class of the dynamic graph manager and that a ros
 * node is NOT created. The ros node is created upon the launch of the thread.
 * During a common initilialization phase.
 */
TEST_F(TestDynamicGraphManager, test_hwc_process_node_handle_exists)
{
    /** Setup */
    SimpleDGM dgm;
    dgm.initialize(params_);
    dgm.initialize_hardware_communication_process();

    /** Test */
    ASSERT_TRUE(ros_node_exists(dgm.hw_com_ros_node_name_));
}

/**
 * Here we check that upon initialization, the hardware communication thread is
 * not running
 */
TEST_F(TestDynamicGraphManager, test_hwc_process_stopped_on_init)
{
    /** Setup */
    SimpleDGM dgm;
    dgm.initialize(params_);
    dgm.initialize_hardware_communication_process();

    /** Test */
    ASSERT_TRUE(dgm.is_hardware_communication_stopped());
}

/**
 * Check that the dynamic graph is stopped after initialization.
 */
TEST_F(TestDynamicGraphManager, test_dynamic_graph_stopped_upon_start)
{
    /** Setup */
    // Create a dynamic_graph_manager (DGM)
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    // Initialize dg process
    dgm.initialize_dynamic_graph_process();

    /** Test */
    ASSERT_TRUE(dgm.is_dynamic_graph_stopped());
}

/**
 * Here we check that if the "/dynamic_graph/start_dynamic_graph" ros service
 * works properly.
 */
TEST_F(TestDynamicGraphManager, test_start_dynamic_graph_ros_services)
{
    /** Setup */
    // Create a dynamic_graph_manager (DGM)
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    // Initialize dg process
    dgm.initialize_dynamic_graph_process();
    // Create a client from a temporary node
    start_dg_ros_service();

    /** Test */
    ASSERT_FALSE(dgm.is_dynamic_graph_stopped());
}

/**
 * Here we check that if the "/dynamic_graph/stop_dynamic_graph" ros service
 * works properly.
 */
TEST_F(TestDynamicGraphManager, test_stop_dynamic_graph_ros_services)
{
    /** Setup */
    // Create a dynamic_graph_manager (DGM)
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    // Initialize dg process
    dgm.initialize_dynamic_graph_process();
    // Start the dynamic graph
    dgm.start_dynamic_graph();
    // Create a client from a temporary node
    stop_dg_ros_service();

    /** Test */
    ASSERT_TRUE(dgm.is_dynamic_graph_stopped());
}

/**
 * Here we check that if the method start_dynamic_graph works properly.
 */
TEST_F(TestDynamicGraphManager, test_start_dynamic_graph_methods)
{
    /** Setup */
    // Create a dynamic_graph_manager (DGM)
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    // Initialize dg process
    dgm.initialize_dynamic_graph_process();
    // Start the dynamic graph
    dgm.start_dynamic_graph();

    /** Test */
    ASSERT_FALSE(dgm.is_dynamic_graph_stopped());
}

/**
 * Here we check that if the method stop_dynamic_graph works properly.
 */
TEST_F(TestDynamicGraphManager, test_stop_dynamic_graph_methods)
{
    /** Setup */
    // Create a dynamic_graph_manager (DGM)
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    // Initialize dg process
    dgm.initialize_dynamic_graph_process();
    // Start the dynamic graph
    dgm.start_dynamic_graph();
    // Stop it again :)
    dgm.stop_dynamic_graph();

    /** Test */
    ASSERT_TRUE(dgm.is_dynamic_graph_stopped());
}

void start_dg_method(DynamicGraphManager* dgm)
{
    sleep(0.05);
    dgm->start_dynamic_graph();
}

/**
 * @brief Test the methods that put the current thread to sleep until the
 * method "start_dynamic_graph" is called.
 */
TEST_F(TestDynamicGraphManager, test_wait_start_dynamic_graph_method)
{
    /** Setup */
    // first we create the dynamic graph process
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();
    // then we launch a thread that will call the start dynamic graph
    std::thread start_dg_thread = std::thread(&start_dg_method, &dgm);
    // Pause the current thread
    dgm.wait_start_dynamic_graph();
    // Wait until the thread is finished
    start_dg_thread.join();

    /** Test */
    ASSERT_FALSE(dgm.is_dynamic_graph_stopped());
}

/**
 * @brief Test the methods that put the current thread to sleep until the
 * rosservice "dynamic_graph/start_dynamic_graph" is called.
 */
TEST_F(TestDynamicGraphManager, test_wait_start_dynamic_graph_rosservice)
{
    /** Setup */
    // first we create the dynamic graph process
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();
    // then we launch a thread that will call the start dynamic graph
    std::thread start_dg_thread = std::thread(&start_dg_ros_service);
    // Pause the current thread
    dgm.wait_start_dynamic_graph();
    // Wait until the thread is finished
    start_dg_thread.join();

    /** Test */
    ASSERT_FALSE(dgm.is_dynamic_graph_stopped());
}

/**
 * @brief Call start and stop dynamic graph from methods
 *
 * @param a dynamic graph manager
 */
void manage_dg_process(DynamicGraphManager* dgm)
{
    // wait a bit so the main thread is executed
    sleep(1.0);

    // start the dynamic graph.
    dgm->start_dynamic_graph();

    // Sleep a bit so the thread is launched and can spin once
    sleep(2);

    // stop the dynamic graph
    dgm->stop_dynamic_graph();
}

/**
 * This test runs the dynamic graph process completely
 */
TEST_F(TestDynamicGraphManager, test_run_dynamic_graph_process)
{
    /** Setup */

    // Initialize the dynamic graph manager
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();

    // Modify the ouput torques
    dynamicgraph::Vector vec;
    vec.resize(5);
    vec.fill(0.0);
    vec(0) = 1.0;
    dgm.device().motor_controls_in_.at("torques")->setConstant(vec);

    // then we launch a thread that will call the start dynamic graph
    std::thread start_dg_thread = std::thread(&manage_dg_process, &dgm);
    // launch the process and wait it to start
    dgm.run_dynamic_graph_process();
    // Once started, wait for it to stop
    dgm.wait_stop_dynamic_graph();
    // join the thread
    start_dg_thread.join();

    /** Test */
    double norm_torques = dgm.device().motor_controls_map_.at("torques").norm();
    ASSERT_DOUBLE_EQ(norm_torques, 1.0);
}

/**
 * @brief Test if the start and stop of the hardware communication thread
 * is done nicely
 */
TEST_F(TestDynamicGraphManager, test_run_hardware_communication_process)
{
    /** Setup */
    SimpleDGM dgm;
    dgm.initialize(params_);
    dgm.initialize_hardware_communication_process();
    dgm.run_hardware_communication_process();
    usleep(5000);

    /** Test */
    ASSERT_FALSE(dgm.is_hardware_communication_stopped());

    /** Some other setup */
    dgm.stop_hardware_communication();
    dgm.wait_stop_hardware_communication();

    /** Final test */
    ASSERT_TRUE(dgm.is_hardware_communication_stopped());
}
