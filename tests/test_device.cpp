/**
 * \file test_device.cpp
 * \brief Device unit tests
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file implements a suit of unit tests for the Device class.
 * @see https://git-amd.tuebingen.mpg.de/amd-clmc/ci_example/wikis/catkin:-how-to-implement-unit-tests
 */

#include <ostream>
#include <cstdlib>      // std::rand, std::srand
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/factory.h>
#include <dynamic_graph_manager/device.hh>

using namespace dynamic_graph;
namespace dg = dynamicgraph;

/**
 * @brief The DISABLED_TestDevice class is used to disable test.
 */
class DISABLED_TestDevice : public ::testing::Test {};

/**
 * @brief The TestDevice class: test suit template for setting up
 * the unit tests for the Device.
 */
class TestDevice : public ::testing::Test {

public:
   TestDevice(): ::testing::Test(),
     dg_pool_(dg::g_pool()),
     entity_map_(dg_pool_.getEntityMap())
   {}

protected:
  /**
   * @brief SetUp, is executed before the unit tests
   */
  void SetUp() {
    dg_pool_.getInstance();
    YAML::Node params = YAML::LoadFile(TEST_CONFIG_PATH +
                            std::string("simple_robot.yaml"));
    params_ = params["device"];
  }

  /**
   * @brief TearDown, is executed after teh unit tests
   */
  void TearDown() {
    dg_pool_.destroy();
  }

  YAML::Node params_;
  YAML::Node empty_params_;

  dynamicgraph::PoolStorage& dg_pool_;
  const dg::PoolStorage::Entities& entity_map_;
};

/**
 * @brief test_start_stop_ros_services, test the start/stop dynamic graph ROS
 * services
 */
TEST_F(TestDevice, test_get_class_name)
{
  Device device("banana");
  device.initialize(empty_params_);
  ASSERT_EQ("Device", device.CLASS_NAME);
  ASSERT_EQ("Device", device.getClassName());
  ASSERT_EQ("banana", device.getName());
}

TEST_F(TestDevice, test_constructor)
{
  Device device("banana");
  device.initialize(empty_params_);
  ASSERT_EQ(device.sensors_map_.size(), 0);
  ASSERT_EQ(device.sensors_out_.size(), 0);
  ASSERT_EQ(device.motor_controls_map_.size(), 0);
  ASSERT_EQ(device.motor_controls_in_.size(), 0);
}

TEST_F(TestDevice, test_destructor)
{
  {
    Device device("banana");
    device.initialize(empty_params_);
    ASSERT_EQ(entity_map_.count("banana"), 1);
  }
  ASSERT_EQ(entity_map_.count("banana"), 0);
}

TEST_F(TestDevice, test_parse_yaml_file)
{
  Device device("simple_robot");
  device.initialize(params_);
//  for(VectorDGMap::const_iterator it=device.sensors_map_.begin() ;
//      it!=device.sensors_map_.end() ; ++it)
//  {
//    std::cout << it->first << " ; " << it->second.transpose() << std::endl;
//  }
//  for(VectorDGMap::const_iterator it=device.motor_controls_map_.begin() ;
//      it!=device.motor_controls_map_.end() ; ++it)
//  {
//    std::cout << it->first << " ; " << it->second.transpose() << std::endl;
//  }
//  for(DeviceOutSignalMap::const_iterator it=device.sensors_out_.begin() ;
//      it!=device.sensors_out_.end() ; ++it)
//  {
//    std::cout << it->first << " ; " << it->second->accessCopy().transpose() << std::endl;
//  }
//  for(DeviceInSignalMap::const_iterator it=device.motor_controls_in_.begin() ;
//      it!=device.motor_controls_in_.end() ; ++it)
//  {
//    std::cout << it->first << " ; " << it->second->accessCopy().transpose() << std::endl;
//  }
  const Device::SignalMap& sig_map = device.getSignalMap();
  ASSERT_EQ(sig_map.count("encoders"), 1);
  ASSERT_EQ(sig_map.count("imu_accelerometer"), 1);
  ASSERT_EQ(sig_map.count("imu_gyroscope"), 1);
  ASSERT_EQ(sig_map.count("imu"), 1);
  ASSERT_EQ(sig_map.count("torques"), 1);
  ASSERT_EQ(sig_map.count("positions"), 1);
  ASSERT_EQ(device.sensors_out_["encoders"]->accessCopy().size(), 5);
  ASSERT_EQ(device.sensors_out_["imu_accelerometer"]->accessCopy().size(), 3);
  ASSERT_EQ(device.sensors_out_["imu_gyroscope"]->accessCopy().size(), 3);
  ASSERT_EQ(device.sensors_out_["imu"]->accessCopy().size(), 6);
  ASSERT_EQ(device.motor_controls_in_["torques"]->accessCopy().size(), 5);
  ASSERT_EQ(device.motor_controls_in_["positions"]->accessCopy().size(), 5);
}

TEST_F(TestDevice, test_set_sensors_from_map)
{
  // create the device
  Device device("simple_robot");
  device.initialize_maps(params_);

  // prepare some randomness
  srand(static_cast<unsigned>(time(nullptr)));
  std::vector<double> rand_vec;

  // create the sensors map
  VectorDGMap sensors;

  // fill in the sensor map
  sensors["encoders"] = dg::Vector::Random(5);
  sensors["imu_accelerometer"] = dg::Vector::Random(3);
  sensors["imu_gyroscope"] = dg::Vector::Random(3);
  sensors["imu"] = dg::Vector::Random(6);

  device.set_sensors_from_map(sensors);

  ASSERT_TRUE(sensors["encoders"].isApprox(
        device.sensors_out_["encoders"]->accessCopy()));
  ASSERT_TRUE(sensors["imu_accelerometer"].isApprox(
        device.sensors_out_["imu_accelerometer"]->accessCopy()));
  ASSERT_TRUE(sensors["imu_gyroscope"].isApprox(
        device.sensors_out_["imu_gyroscope"]->accessCopy()));
  ASSERT_TRUE(sensors["imu"].isApprox(
        device.sensors_out_["imu"]->accessCopy()));
}

class SimpleRobot: public Device
{
public:
  static const std::string CLASS_NAME;
  SimpleRobot(std::string RobotName):
    Device(RobotName)
  {
    initialize(YAML::LoadFile(TEST_CONFIG_PATH + std::string("simple_robot.yaml")));
    sig_before_.reset(new OutSignal("SimpleRobot::before::sigbefore"));
    sig_after_.reset(new OutSignal("SimpleRobot::after::sigafter"));
    sig_before_->setConstant(dg::Vector(3));
    sig_after_->setConstant(dg::Vector(3));
    signalRegistration(*sig_after_ << *sig_before_);
    periodic_call_before_.addSignal(name + ".sigbefore");
    periodic_call_after_.addSignal(name + ".sigafter");
  }
  std::unique_ptr<OutSignal> sig_before_;
  std::unique_ptr<OutSignal> sig_after_;
};

TEST_F(TestDevice, test_execute_graph)
{
  // create the device
  SimpleRobot device("simple_robot");
  device.initialize_maps(params_);

  // setup the controls
  device.motor_controls_in_["torques"]->setConstant(dg::Vector::Random(5));
  device.motor_controls_in_["positions"]->setConstant(dg::Vector::Random(5));

  ASSERT_EQ(device.sig_after_->getTime(), device.sig_before_->getTime());
  double time_before = device.sig_after_->getTime();

  device.execute_graph();
  ASSERT_EQ(device.sig_after_->getTime(), device.sig_before_->getTime());
  ASSERT_EQ(device.sig_after_->getTime(), time_before + 1);
  ASSERT_EQ(device.sig_before_->getTime(), time_before + 1);
}

TEST_F(TestDevice, test_get_controls_to_map)
{
  // create the device
  Device device("simple_robot");
  device.initialize_maps(params_);

  // prepare some randomness
  srand(static_cast<unsigned>(time(nullptr)));
  std::vector<double> rand_vec;

  // create the controls map
  VectorDGMap motor_controls;
  motor_controls["torques"].resize(5);
  motor_controls["positions"].resize(5);

  // setup the controls
  device.motor_controls_in_["torques"]->setConstant(dg::Vector::Random(5));
  device.motor_controls_in_["positions"]->setConstant(dg::Vector::Random(5));

  device.get_controls_to_map(motor_controls);

  Eigen::Map<dg::Vector> torques(&motor_controls["torques"][0], 5);
  Eigen::Map<dg::Vector> positions(&motor_controls["positions"][0], 5);

  ASSERT_TRUE(torques.isApprox(
                device.motor_controls_in_["torques"]->accessCopy()));
  ASSERT_TRUE(positions.isApprox(
                device.motor_controls_in_["positions"]->accessCopy()));
}

