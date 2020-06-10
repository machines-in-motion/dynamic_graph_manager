/**
 * @file test_cpp_features.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <gtest/gtest.h>
#include <map>

/**
 * @brief The TestCppFeatures class: test suit template for setting up
 * the unit tests for testing cpp features
 */
class TestCppFeatures : public ::testing::Test
{
protected:
    /**
     * @brief SetUp, is executed before the unit tests
     */
    void SetUp()
    {
    }

    /**
     * @brief TearDown, is executed after the unit tests
     */
    void TearDown()
    {
    }
};

// 14
TEST_F(TestCppFeatures, test_exit_fork)
{
    pid_t pid_child = 0;
    // pid_t pid_parent = 0;
    pid_t pid = fork();
    if (pid == 0)  // Child process
    {
        pid_child = getpid();
        // pid_parent = getppid();
        //    std::cout << "child: pid_child=" << pid_child <<
        //                 " ; pid_parent=" << pid_parent << std::endl;
        usleep(5000);
        exit(0);
    }
    else if (pid > 0)  // Parent process
    {
        pid_child = pid;
        // pid_parent = getpid();
        //    std::cout << "parent: pid_child=" << pid_child <<
        //                 " ; pid_parent=" << pid_parent << std::endl;
        bool child_death_detected = false;
        int status;
        pid_t p;
        p = waitpid(pid_child, &status, WNOHANG);
        if (p == 0)
        {
            child_death_detected = false;
        }
        else if (p > 0)
        {
            child_death_detected = true;
        }
        else
        {
            ASSERT_TRUE(false && "waitpid failed");
        }
        ASSERT_FALSE(child_death_detected);

        usleep(200000);
        p = waitpid(pid_child, &status, WNOHANG);
        //    std::cout << "p=" << p << std::endl;
        if (p == 0)
        {
            child_death_detected = false;
        }
        else if (p > 0)
        {
            child_death_detected = true;
        }
        else
        {
            ASSERT_TRUE(false && "waitpid failed");
        }
        ASSERT_TRUE(child_death_detected);
        wait(nullptr);
    }
    else
    {
        ASSERT_TRUE(false && "fork failed");
    }
}

TEST_F(TestCppFeatures, test_on_map_and_pointers)
{
    typedef std::map<std::string, double*> map_data;
    map_data data;
    data["first"] = new double(1.0);
    data["second"] = new double(2.0);
    data["third"] = new double(3.0);
    data["fourth"] = new double(4.0);
    data["fifth"] = new double(5.0);
    for (map_data::iterator it = data.begin(); it != data.end(); ++it)
    {
        //    std::cout << data.size() << " ; "
        //              << it->first << " " << *(it->second)
        //              << std::endl;
        double* d = it->second;
        data.erase(it);
        delete d;
    }
    //  std::cout << "******" << std::endl;
    data["first"] = new double(1.0);
    data["second"] = new double(2.0);
    data["third"] = new double(3.0);
    data["fourth"] = new double(4.0);
    data["fifth"] = new double(5.0);
    for (map_data::iterator it = data.begin(); it != data.end();
         it = data.begin())
    {
        //    std::cout << data.size() << " ; "
        //              << it->first << " " << *(it->second)
        //              << std::endl;
        double* d = it->second;
        data.erase(it);
        delete d;
    }
}
