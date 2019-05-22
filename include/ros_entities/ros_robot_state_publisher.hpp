/**
 * @file ros_robot_state_publisher.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 * @date 2019-05-22
 */

#ifndef ROS_ROBOT_STATE_PUBLISHER_HPP
#define ROS_ROBOT_STATE_PUBLISHER_HPP

#include <deque>
#include <memory>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/linear-algebra.h>

# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
# include <tf2_msgs/TFMessage.h>
# include <realtime_tools/realtime_publisher.h>

/**
 * @brief this is this package namespace in order to avoid naming conflict
 */
namespace dynamic_graph
{

/**
 * @brief Simple shortcut for code writing convenience
 */
typedef dynamicgraph::SignalTimeDependent<int,int> SignalOUT;
/**
 * @brief Simple shortcut for code writing convenience
 */
typedef dynamicgraph::SignalPtr<dynamicgraph::Vector, int> SignalIN;
/**
 * @brief Simple shortcut for code writing convenience
 */
typedef boost::function<void (int)> callback_t;

/**
 * @brief Renaming of the tf publisher type
 */
typedef realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>
    TfRtPublisher;

/**
 * @brief Renaming of the joint position publisher type
 */
typedef realtime_tools::RealtimePublisher <sensor_msgs::JointState> 
    JointStateRtPublisher;

struct RosRobotStatePublisherInternal
{
  std::shared_ptr<TfRtPublisher> base_tf_publisher_;
  std::shared_ptr<JointStateRtPublisher> joint_state_publisher_;
  std::shared_ptr<SignalIN> robot_state_input_signal_;
  callback_t callback_function_;
};

/**
 * @brief This class define a dynamic graph wrapper around the vicon client
 */
class RosRobotStatePublisher: public dynamicgraph::Entity
{
public:
  
  RosRobotStatePublisher( const std::string& name );

  ~RosRobotStatePublisher( void ){}

  /**
   * @brief 
   * 
   * @param base_link_name 
   * @param joint_names 
   * @param tf_prefix 
   * @param signal_name 
   */
  void add(const std::string& base_link_name,
           const std::string& joint_names,
           const std::string& tf_prefix,
           const std::string& signal_name,
           const std::string& joint_state_topic_name);

  /**
   * @brief Trigger the publishing of the data to ros for all signals.
   * 
   * @return int& dummy stuff.
   */
  int& trigger(int&, int);

  /**
   * @brief Signal callback functions.
   */
  void send_data(const RosRobotStatePublisherInternal& publisher,
                 int time);

private:
  /**
   * @brief The class name used to identify it in the dynamic graph pool
   */
  static const std::string CLASS_NAME;

  /**
   * @brief Get the CLASS_NAME object
   * 
   * @return const std::string& 
   */
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  /**
   * @brief Set a tf msg to identity.
   * 
   * @param[in][out] The msg to reset to indentity
   */
  void set_tf_msg_to_identity(geometry_msgs::TransformStamped& msg);

  /**
   * @brief Normalize the tf message quaternion before sending it to the 
   * 
   * @param[in][out] the msg to normalize 
   */
  void normalize_tf_msg_quaternion(geometry_msgs::TransformStamped& msg);

  /**
   * @brief Create an internal output signal which should be call periodically
   * in the device in order to publish the data. The callback function of this
   * signal is the trigger methode.
   */
  SignalOUT trigger_signal_;

  /**
   * @brief Ros node handle corresponding to the dynamic graph process node
   * handle.
   */
  ros::NodeHandle& ros_node_handle_;

  /**
   * @brief This is the list of publishers registered for this class. It
   * correspond to the list of robot states one wants to publish. See the add
   * methode (Command)
   */
  std::map<std::string, RosRobotStatePublisherInternal> publishers_;

  /**
   * @brief manage the rate data publication in topics
   */
  ros::Duration rate_;

  /**
   * @brief Check at what time the publication was done
   */
  ros::Time last_time_of_publication_;

};
} // namespace dynamic_graph


namespace dynamicgraph
{
/**
 * @brief this is this the namespace including the commands
 */
namespace command
{
/**
 * @brief this is this a namespace including the ros state publisher command
 */
namespace ros_state_publish
{
  using ::dynamicgraph::command::Command;
  using ::dynamicgraph::command::Value;

  /**
   * Declare here a couple of python command used for the ros_state publisher.
   */

  // Define here a macro that automatically generates a command
  # define ROS_PUBLISH_MAKE_COMMAND(CMD)                         \
  class CMD : public Command                                     \
  {                                                              \
    public:                                                      \
      CMD (dynamic_graph::RosRobotStatePublisher& entity,        \
           const std::string& docstring);                        \
      virtual Value doExecute ();                                \
  }

  // Generate a couple of command classes
  ROS_PUBLISH_MAKE_COMMAND(Add);
  // Later in the code no one must have access to this macro.
  #undef ROS_PUBLISH_MAKE_COMMAND
} // end of namespace ros_state_publish
} // end of namespace command
} // end of namespace dynamicgraph


#endif // ROS_ROBOT_STATE_PUBLISHER_HPP