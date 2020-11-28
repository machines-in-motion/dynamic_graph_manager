/**
 * @file test_dynamic_graph_manager.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

// Common include for all tests
#include "test_common.hpp"

/**
 * SETUP THE TESTING CLASS
 */

/**
 * @brief The DISABLED_TestDynamicGraphManagerControl class is used to disable
 * test
 */
class DISABLED_TestDynamicGraphManagerControl : public ::testing::Test
{
public:
    YAML::Node params_;
};

/**
 * @brief The TestDynamicGraphManagerControl class: test suit template for
 * setting up the unit tests for the DynamicGraphManager
 */
class TestDynamicGraphManagerControl : public ::testing::Test
{
protected:
    /**
     * @brief SetUp, is executed before the unit tests
     */
    void SetUp()
    {
        ros_clean();
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
        ros_clean();
    }

    /**
     * @brief Params of the DGM
     */
    YAML::Node params_;
};

/*****************
 * Start Testing *
 *****************/

/**
 * Here we check that the initialization of the dynamic graph process creates
 * the ros node properly.
 */
TEST_F(TestDynamicGraphManagerControl, test_init_dg_process_node_handle_exists)
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
TEST_F(TestDynamicGraphManagerControl, test_init_dg_process_start_dg_exist)
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
TEST_F(TestDynamicGraphManagerControl, test_init_dg_process_stop_dg_exist)
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
TEST_F(TestDynamicGraphManagerControl,
       test_init_dg_process_run_python_cmd_exist)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();
    RosNodePtr ros_node = get_ros_node(dgm.dg_ros_node_name_);
    auto client = ros_node->create_client<mim_msgs::srv::RunPythonCommand>(
        "/dynamic_graph_manager/run_python_command");

    /** Test */
    ASSERT_TRUE(client->wait_for_service(1s));
}

/**
 * Here we check that the initialization of the dynamic graph process runs
 * correctly. The "/dynamic_graph/run_python_script" ros sservice must be
 * available by now.
 */
TEST_F(TestDynamicGraphManagerControl,
       test_init_dg_process_run_python_file_exist)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    dgm.initialize_dynamic_graph_process();
    RosNodePtr ros_node = get_ros_node(dgm.dg_ros_node_name_);
    auto client = ros_node->create_client<mim_msgs::srv::RunPythonFile>(
        "/dynamic_graph_manager/run_python_file");

    /** Test */
    ASSERT_TRUE(client->wait_for_service(1s));
}

/**
 * Check that the dynamic graph is stopped after initialization.
 */
TEST_F(TestDynamicGraphManagerControl, test_dynamic_graph_stopped_upon_start)
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
TEST_F(TestDynamicGraphManagerControl, test_start_dynamic_graph_ros_services)
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
TEST_F(TestDynamicGraphManagerControl, test_stop_dynamic_graph_ros_services)
{
    /** Setup */
    // Create a dynamic_graph_manager (DGM)
    DynamicGraphManager dgm;
    dgm.initialize(params_);
    // Initialize dg process
    dgm.initialize_dynamic_graph_process();
    // Start the dynamic graph
    dgm.start_dynamic_graph();
    // Call the stop_dynamic_graph ros_service.
    stop_dg_ros_service();

    /** Test */
    ASSERT_TRUE(dgm.is_dynamic_graph_stopped());
}

/**
 * Here we check that if the method start_dynamic_graph works properly.
 */
TEST_F(TestDynamicGraphManagerControl, test_start_dynamic_graph_methods)
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
TEST_F(TestDynamicGraphManagerControl, test_stop_dynamic_graph_methods)
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
TEST_F(TestDynamicGraphManagerControl, test_wait_start_dynamic_graph_method)
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
TEST_F(TestDynamicGraphManagerControl, test_wait_start_dynamic_graph_rosservice)
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
    real_time_tools::Timer::sleep_sec(0.1);

    // start the dynamic graph.
    std::cout << "test_dgm_control/manage_dg_process(): start dynamic graph"
              << std::endl;
    dgm->start_dynamic_graph();
    // We need to activate the loop to candence it.
    shared_memory::LockedConditionVariable dgm_cond_var("cond_var", false);
    dgm_cond_var.lock_scope();

    // Sleep a bit so the thread is launched and can spin 10 times exactly
    for(unsigned int i = 0 ; i < 10 ; ++i)
    {
        dgm_cond_var.notify_all();
        dgm_cond_var.wait();
        real_time_tools::Timer::sleep_sec(0.01);
    }

    // stop the dynamic graph
    std::cout << "test_dgm_control/manage_dg_process(): stop dynamic graph"
              << std::endl;
    dgm->stop_dynamic_graph();
    dgm_cond_var.notify_all();
}

/**
 * This test runs the dynamic graph process completely
 */
TEST_F(TestDynamicGraphManagerControl, test_run_dynamic_graph_process)
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

    // launch the process and wait it to start
    dgm.run_dynamic_graph_process();

    // then we launch a thread that will call the start dynamic graph
    std::thread start_dg_thread = std::thread(&manage_dg_process, &dgm);

    // Once started, wait for it to stop
    dgm.wait_stop_dynamic_graph();

    // join the thread
    start_dg_thread.join();

    /** Test */
    double norm_torques = dgm.device().motor_controls_map_.at("torques").norm();
    ASSERT_DOUBLE_EQ(norm_torques, 1.0);
    ASSERT_EQ(dgm.device().motor_controls_in_.at("torques")->getTime(), 1);
}

/**
 * Test that we can stop and reboot the dynamic graph process.
 *
 * NOTICE! In RosTest you need to add THE "DISABLED_" also in the test name
 * to be able to disabled completely the test.
 */
TEST_F(DISABLED_TestDynamicGraphManagerControl,
       DISABLED_test_dynamic_graph_re_initialization)
{
    // verify that ros is not active
    ASSERT_FALSE(ros_ok());

    // initialize ROS (not needed here as the dynamic graph manager does it)

    // create the DGM
    DynamicGraphManager dgm;
    dgm.initialize(params_);

    // initialize dgm
    dgm.initialize_dynamic_graph_process();

    // Check that ros is active
    ASSERT_TRUE(ros_ok());

    // check the run_python_command service
    std::string cmd = "1 + 1";
    std::string result = "";
    std::string standard_error = "";
    std::string standard_output = "";
    start_run_python_command_ros_service(
        cmd, result, standard_error, standard_output);
    ASSERT_EQ(result, "2");

    // check the run_script service and the python interpreter
    std::string file_name = TEST_CONFIG_PATH + std::string("simple_add.py");
    result = "";
    standard_error = "";
    start_run_python_script_ros_service(file_name, result, standard_error);

    // check that "a" exists and has a value "3"
    cmd = "a";
    result = "";
    standard_error = "";
    standard_output = "";
    start_run_python_command_ros_service(
        cmd, result, standard_error, standard_output);
    ASSERT_EQ(result, "3");

    // Re-initialize
    dgm.initialize_dynamic_graph_process();

    //  // perform a simple operation
    //  run_com_msg.request.input = "1 + 1";
    //  run_python_command_client.call(run_com_msg);
    //  ASSERT_EQ(run_com_msg.response.result, "False");

    //  // check that "a" does not exists
    //  run_com_msg.request.input = "'a' in globals()";
    //  run_python_command_client.call(run_com_msg);
    //  ASSERT_EQ(run_com_msg.response.result, "False");

    //  run_com_msg.request.input = "'a' in locals()";
    //  run_python_command_client.call(run_com_msg);
    //  ASSERT_EQ(run_com_msg.response.result, "False");
}
