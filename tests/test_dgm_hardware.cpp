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
 * @brief The DISABLED_TestDynamicGraphManagerHardware class is used to disable
 * test
 */
class DISABLED_TestDynamicGraphManagerHardware : public ::testing::Test
{
public:
    YAML::Node params_;
};

/**
 * @brief The TestDynamicGraphManagerHardware class: test suit template for
 * setting up the unit tests for the DynamicGraphManager hardware communication
 * user commands
 */
class TestDynamicGraphManagerHardware : public ::testing::Test
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
 * Here we check that the initialization of the hardware communication fails
 * if we call the methode from the raw dynamic graph manager class.
 */
TEST_F(TestDynamicGraphManagerHardware, test_initialize_hwc_throw)
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
TEST_F(TestDynamicGraphManagerHardware, test_initialize_hwc_no_throw)
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
TEST_F(TestDynamicGraphManagerHardware, test_hwc_process_node_handle_exists)
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
TEST_F(TestDynamicGraphManagerHardware, test_hwc_process_stopped_on_init)
{
    /** Setup */
    SimpleDGM dgm;
    dgm.initialize(params_);
    dgm.initialize_hardware_communication_process();

    /** Test */
    ASSERT_TRUE(dgm.is_hardware_communication_stopped());
}

/**
 * @brief Test if the start and stop of the hardware communication thread
 * is done nicely
 */
TEST_F(TestDynamicGraphManagerHardware, test_run_hardware_communication_process)
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

/**
 * @brief Test if the start and stop of the hardware communication thread
 * is done nicely
 */
TEST_F(TestDynamicGraphManagerHardware, test_run_user_cmd)
{
    /** Setup */
    SimpleDGM dgm;
    dgm.initialize(params_);
    dgm.initialize_hardware_communication_process();
    dgm.run_hardware_communication_process();

    /** Test 1 */
    ASSERT_FALSE(dgm.get_has_user_command_been_executed());

    /** Test 2 */
    user_cmd_ros_service(true);
    ASSERT_TRUE(dgm.get_has_user_command_been_executed());

    /** Test 3 */
    user_cmd_ros_service(false);
    ASSERT_FALSE(dgm.get_has_user_command_been_executed());

    /** Tear down */
    dgm.stop_hardware_communication();
    dgm.wait_stop_hardware_communication();
    ros_shutdown();
}

/**
 * WARNING this has to be the last test as we ask for ros::shutdown
 * Test if ros::shutdown is actually killing the thread
 */
TEST_F(DISABLED_TestDynamicGraphManagerHardware,
       DISABLED_test_hwc_process_ros_shutdown)
{
    /** Setup */
    SimpleDGM dgm;
    dgm.initialize(params_);
    dgm.initialize_hardware_communication_process();
    dgm.run_hardware_communication_process();
    usleep(5000);

    /** First test */
    ASSERT_FALSE(dgm.is_hardware_communication_stopped());

    /** Some more setup */
    ros_shutdown();
    dgm.wait_stop_hardware_communication();

    /** Final test */
    ASSERT_TRUE(dgm.is_hardware_communication_stopped());
}
