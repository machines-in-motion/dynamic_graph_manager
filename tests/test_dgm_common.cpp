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
 * @brief The DISABLED_TestDynamicGraphManagerCommon class is used to disable
 * test
 */
class DISABLED_TestDynamicGraphManagerCommon : public ::testing::Test
{
public:
    YAML::Node params_;
};

/**
 * @brief The TestDynamicGraphManagerCommon class: test suit template for
 * setting up the unit tests for the DynamicGraphManager
 */
class TestDynamicGraphManagerCommon : public ::testing::Test
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
        ros_stop_spinning();
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
 * Here we check that the constructor throw no error
 */
TEST_F(TestDynamicGraphManagerCommon, test_constructor)
{
    /** Test */
    ASSERT_NO_THROW(DynamicGraphManager dgm;);
}

/**
 * Here we check that the destructor throw no error
 */
TEST_F(TestDynamicGraphManagerCommon, test_destructor)
{
    /** Test */
    ASSERT_NO_THROW({ DynamicGraphManager dgm; });
}

/**
 * Here we check that the initialization throw no error
 */
TEST_F(TestDynamicGraphManagerCommon, test_initialize)
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
TEST_F(TestDynamicGraphManagerCommon, test_initialize_no_dg_node_handle)
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
TEST_F(TestDynamicGraphManagerCommon, test_initialize_no_hwc_node_handle)
{
    /** Setup */
    DynamicGraphManager dgm;
    dgm.initialize(params_);

    /** Test */
    ASSERT_FALSE(ros_node_exists(dgm.hw_com_ros_node_name_));
}

/**
 * @brief Test the standard behavior of the DGM until the safety mode is
 * activated
 */
TEST_F(DISABLED_TestDynamicGraphManagerCommon,
       DISABLED_test_complete_run_with_safety_mode_activation)
{
    /** Setup */
    std::cerr << ("main: Setup") << std::endl;
    SimpleDGM dgm;
    dgm.initialize(params_);
    dgm.run();
    // wait that everything started
    usleep(5000);
    // Start the dynamic graph
    start_dg_ros_service();
    std::cerr << ("main: Dynamic Graph Started") << std::endl;
    // wait few iteration of the dynamic_graph
    sleep(1);
    // Stop the dynamic graph
    stop_dg_ros_service();
    std::cerr << ("main: Dynamic Graph Stopped") << std::endl;
    std::cerr << ("main: Wait for the DG process to die") << std::endl;
    kill(dgm.pid_dynamic_graph_process(), SIGKILL);
    while (!dgm.has_dynamic_graph_process_died())
    {
        usleep(1000);
    }
    std::cerr << ("main: Dynamic Graph Process Died") << std::endl;
    std::cerr << ("main: Check that the security mode has been activated")
              << std::endl;

    /** Test */
    ASSERT_TRUE(dgm.is_in_safety_mode());

    /** Tear Down */
    dgm.stop_hardware_communication();
    usleep(5000);
}
