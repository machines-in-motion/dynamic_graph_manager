/**
 * @file simple_dgm.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-04-30
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "dynamic_graph_manager/ros_init.hh"
#include "dynamic_graph_manager/dynamic_graph_manager.hh"
#include "dynamic_graph_manager/TestUserCmdBool.h"

namespace dynamic_graph_manager{
  
/**
 * @brief This class is a simple dynamic graph manager with a fake hardware
 * interface used for unittesting
 */
class SimpleDGM : public dynamic_graph::DynamicGraphManager
{
public:
  /**
   * @brief Construct a new SimpleDGM object
   */
  SimpleDGM(): dynamic_graph::DynamicGraphManager(){
    boolean_set_by_user_cmd_ = false;
  }
  /**
   * @brief Destroy the SimpleDGM object
   */
  ~SimpleDGM(){}
  /**
   * @brief Simple overload doing nothing. We have no hardware here for the unit
   * tests.
   */
  void initialize_hardware_communication_process()
  {
    // get the hardware communication ros node handle
    ros::NodeHandle& ros_node_handle = dynamic_graph::ros_init(
      dynamic_graph::DynamicGraphManager::hw_com_ros_node_name_);
    /** initialize the user commands */
    ros_user_commands_.push_back(ros_node_handle.advertiseService(
        "set_a_boolean", &SimpleDGM::user_command_callback, this));
  }
  /**
   * @brief Get the sensors to the map object.
   * 
   * @param map of sensors
   */
  void get_sensors_to_map(dynamic_graph::VectorDGMap& map)
  {
    map["encoders"].setRandom();
    map["imu_accelerometer"].setRandom();
    map["imu_gyroscope"].setRandom();
    map["imu"].setRandom();
  }
  /**
   * @brief Set the motor controls from map object to no hardware. So nothing to
   * be done here
   * 
   * @param map of controls
   */
  void set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map)
  {/*Nothing to be done*/}

  /**
   * @brief Get the has_user_command_been_executed_ object
   */
  bool get_has_user_command_been_executed()
  {
    // Here we nee to protect the access to this ressource as it may conflict
    // with different thread. Please use cond_var_ to make sure everything
    // is access on its due time.
    bool ret = boolean_set_by_user_cmd_;
    return ret;
  }

  /**
   * @brief This service callback parse the ros messages and register and
   * internal method for further call using the data from the ros message.
   * 
   * @param req this is the user argument
   * @param res this is the feedback of the user command
   * @return true in case the service hase been properly executed
   * @return false in case of failure
   */
  bool user_command_callback(dynamic_graph_manager::TestUserCmdBool::Request& req,
                             dynamic_graph_manager::TestUserCmdBool::Response& res)
  {
    // parse and register the command for further call.
    add_user_command(std::bind(&SimpleDGM::user_command, 
                     this, req.input_boolean));
    // return whatever the user want
    res.sanity_check = true;
    
    // the service has been executed properly
    return true;
  }

private:
  /**
   * @brief The actuall user command called in the real time thread
   * 
   * @param user_input is some boolean
   */
  void user_command(bool user_input)
  {
    // do something with the internal state or with the hardware
    boolean_set_by_user_cmd_ = user_input;
  }
  // some internal hardware class or obect. Here just a simple boolean for
  // unit testing
  std::atomic_bool boolean_set_by_user_cmd_;
};

} //namespace