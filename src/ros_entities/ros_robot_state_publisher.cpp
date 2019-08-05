/**
 * @file ros_robot_state_publisher.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-05-22
 */

#include <iostream>
#include <tf/tf.h>
#include "dynamic_graph_manager/ros_init.hh"
#include "dynamic_graph_manager/dynamic_graph_manager.hh"
#include "ros_entities/converter.hh"
#include "ros_entities/ros_robot_state_publisher.hpp"

using namespace std;
// using namespace dynamicgraph::sot;
using namespace dynamicgraph;

namespace dynamic_graph
{

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosRobotStatePublisher, "RosRobotStatePublisher");

RosRobotStatePublisher::RosRobotStatePublisher( const std::string& name ):
  Entity( name ),
  trigger_signal_ (boost::bind (&RosRobotStatePublisher::trigger, this, _1, _2),
            dynamicgraph::sotNOSIGNAL,
            MAKE_SIGNAL_STRING(name, true, "int", "trigger")),
  ros_node_handle_(ros_init(DynamicGraphManager::dg_ros_node_name_))
{
  // Define the refresh signal as always ready.
  signalRegistration(trigger_signal_);
  trigger_signal_.setNeedUpdateFromAllChildren (true);
  
  // Initialize the publishing rate management mechanism
  try {
    last_time_of_publication_ = ros::Time::now ();
  } catch (const std::exception& exc) {
    throw std::runtime_error ("Failed to call ros::Time::now ():" +
                              std::string (exc.what ()));
  }
  
  // Commands:
  std::string cmd_name;
  std::string docstring;

  // Commands - add
  cmd_name = "add";
  docstring =
        "\n"
        "Create a new ros topic that will publish the state of robot and its "
        "base.\n"
        "The input data is a generalized coordinates configuration vector.\n"
        "The robot tf are published in the /dynamic_graph/world FixedFrame\n"
        "\n"
        "  Input:\n"
        "    - type: string: base link name\n"
        "    - type: string: joint names in one string separated with space\n"
        "    - type: string: tf_prefix used for the robot_state_publisher\n"
        "    - type: string: signal_name\n"
        "    - type: string: joint_state_topic_name\n"
        "\n";
  addCommand (cmd_name, new command::ros_state_publish::Add(*this, docstring));

  // this way we publish as fast as the dynamic_graph process
  rate_ = ros::Duration(0.);
}

void RosRobotStatePublisher::add(const std::string& base_link_name,
                                 const std::string& joint_names,
                                 const std::string& tf_prefix,
                                 const std::string& signal_name,
                                 const std::string& joint_state_topic_name)
{
    // Create an internal structure that own the publisher pointers:
    ////////////////////////////////////////////////////////////////

    publishers_[joint_state_topic_name] = RosRobotStatePublisherInternal();
    // create shortcuts
    RosRobotStatePublisherInternal& pub = publishers_[joint_state_topic_name];

    // Initialize the internal structure:
    /////////////////////////////////////

    // Create the tf publisher
    pub.base_tf_publisher_ = std::make_shared<TfRtPublisher>(
      ros_node_handle_, "/tf", 1);
    pub.base_tf_publisher_->msg_.transforms.emplace_back();
    makeHeader(pub.base_tf_publisher_->msg_.transforms[0].header);
    std::string child_frame_id = tf_prefix + "/" + base_link_name;
    pub.base_tf_publisher_->msg_.transforms[0].child_frame_id = 
      tf::resolve(tf_prefix, base_link_name);
    pub.base_tf_publisher_->msg_.transforms[0].transform.translation.x = 0.0;
    pub.base_tf_publisher_->msg_.transforms[0].transform.translation.y = 0.0;
    pub.base_tf_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
    pub.base_tf_publisher_->msg_.transforms[0].transform.rotation.x = 0.0;
    pub.base_tf_publisher_->msg_.transforms[0].transform.rotation.y = 0.0;
    pub.base_tf_publisher_->msg_.transforms[0].transform.rotation.z = 0.0;
    pub.base_tf_publisher_->msg_.transforms[0].transform.rotation.w = 1.0;

    // Create the joint state publisher
    pub.joint_state_publisher_ = std::make_shared<JointStateRtPublisher>(
      ros_node_handle_, joint_state_topic_name, 1);
    // Get the joint name list
    std::istringstream iss(joint_names);
    std::vector<std::string> joint_names_list(
      (std::istream_iterator<std::string>(iss)),
      std::istream_iterator<std::string>()
    );

    pub.joint_state_publisher_->msg_.name = joint_names_list;
    pub.joint_state_publisher_->msg_.position.resize(joint_names_list.size());
    pub.joint_state_publisher_->msg_.velocity.resize(joint_names_list.size());
    pub.joint_state_publisher_->msg_.effort.resize(joint_names_list.size());
    makeHeader(pub.joint_state_publisher_->msg_.header);

    // Initialize the signal.
    ostringstream oss_signal_name;
    oss_signal_name << "RosRobotStatePublisher(" << this->name << ")::"
                    << "input(vector" << joint_names_list.size() << "d)::"
                    << signal_name ;
    pub.robot_state_input_signal_ = std::make_shared<SignalIN>(
      nullptr, oss_signal_name.str()
    );
    // set default value for the signal
    Vector v;
    v.resize (0);
    pub.robot_state_input_signal_->setConstant (v);
    // register the signal in the dynamic graph
    signalRegistration (*pub.robot_state_input_signal_);

    // Initialize the callback.
    pub.callback_function_ = boost::bind(
      &RosRobotStatePublisher::send_data, this,
        publishers_[joint_state_topic_name],  _1);
    
}

int& RosRobotStatePublisher::trigger(int& dummy, int time)
{
  // We call all the callback function: (send_data) corresponding to the
  // registered topic to send
  typedef std::map<std::string, RosRobotStatePublisherInternal>::iterator
    PubItType;

  // We publish as fast as we can
  // ros::Duration dt = ros::Time::now () - last_time_of_publication_;
  // if (dt < rate_)
  // {
  //   return dummy;
  // }
  // last_time_of_publication_ = ros::Time::now();

  for (PubItType pub_it = publishers_.begin () ; pub_it != publishers_.end () ;
       ++pub_it)
  {
    pub_it->second.callback_function_(time);
  }

  return dummy;
}

void RosRobotStatePublisher::send_data(
  const RosRobotStatePublisherInternal& pub,
  int time)
{
  // some shortcuts
  geometry_msgs::TransformStamped& base_pose_msg = 
    pub.base_tf_publisher_->msg_.transforms[0];
  sensor_msgs::JointState& joint_state_msg = 
    pub.joint_state_publisher_->msg_;

  // acquiere the input signal
  const Vector& robot_state = pub.robot_state_input_signal_->access(time);
  int joint_state_size = joint_state_msg.position.size();

  assert(robot_state.size() == joint_state_size ||
         robot_state.size() == joint_state_size + 7 &&
         "Input signal has the wrong size. The expected size is: number joint, or"
         " the number of joint + 7 (base pose and base quaternion)");

  int index = 0;
  // If the base pose is in the robot state we update the base pose msg
  if(robot_state.size() == joint_state_size + 7)
  {
    if (pub.base_tf_publisher_->trylock ())
    {
      base_pose_msg.header.stamp = ros::Time::now ();
      base_pose_msg.transform.translation.x = robot_state(0);
      base_pose_msg.transform.translation.y = robot_state(1);
      base_pose_msg.transform.translation.z = robot_state(2);
      base_pose_msg.transform.rotation.x    = robot_state(3);
      base_pose_msg.transform.rotation.y    = robot_state(4);
      base_pose_msg.transform.rotation.z    = robot_state(5);
      base_pose_msg.transform.rotation.w    = robot_state(6);
      normalize_tf_msg_quaternion(base_pose_msg);
      pub.base_tf_publisher_->unlockAndPublish ();
    }
    index += 7;
  // Else we set the base pose to the origin
  }else{
    if (pub.base_tf_publisher_->trylock ())
    {
      base_pose_msg.header.stamp = ros::Time::now ();
      set_tf_msg_to_identity(base_pose_msg);
      pub.base_tf_publisher_->unlockAndPublish ();
    }
  }

  if(pub.joint_state_publisher_->trylock())
  {
    pub.joint_state_publisher_->msg_.header.stamp = ros::Time::now ();
    for(int i=0 ; i<pub.joint_state_publisher_->msg_.position.size() ; ++i)
    {
      pub.joint_state_publisher_->msg_.position[i] = robot_state(index + i);
    }
    pub.joint_state_publisher_->unlockAndPublish ();
  }
}

void RosRobotStatePublisher::set_tf_msg_to_identity(
  geometry_msgs::TransformStamped& msg)
{
  msg.transform.translation.x = 0.0;
  msg.transform.translation.y = 0.0;
  msg.transform.translation.z = 0.0;
  msg.transform.rotation.x = 0.0;
  msg.transform.rotation.y = 0.0;
  msg.transform.rotation.z = 0.0;
  msg.transform.rotation.w = 1.0;
}

void RosRobotStatePublisher::normalize_tf_msg_quaternion(
  geometry_msgs::TransformStamped& msg)
{
  double& x = msg.transform.rotation.x;
  double& y = msg.transform.rotation.y;
  double& z = msg.transform.rotation.z;
  double& w = msg.transform.rotation.w;
  double norm = sqrt(x*x +y*y + z*z + w*w);
  if(norm < 1e-8)
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    w = 1.0;
  }else{
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
  }
}

} // namespace dynamic_graph

/**
 * Definition of the command
 */
namespace dynamicgraph
{
namespace command
{
namespace ros_state_publish
{
  Add::Add(dynamic_graph::RosRobotStatePublisher& entity,
           const std::string& docstring):
    Command(entity, boost::assign::list_of (Value::STRING) (Value::STRING)
      (Value::STRING) (Value::STRING) (Value::STRING), docstring)
  {
  }

  Value Add::doExecute ()
  {
    dynamic_graph::RosRobotStatePublisher& entity =
        static_cast<dynamic_graph::RosRobotStatePublisher&> (owner ());
    std::vector<Value> values = getParameterValues ();
    const std::string& base_link_name = values[0].value ();
    const std::string& joint_names = values[1].value ();
    const std::string& tf_prefix = values[2].value ();
    const std::string& signal_name = values[3].value ();
    const std::string& joint_state_topic_name = values[4].value ();
    entity.add(base_link_name,
               joint_names,
               tf_prefix,
               signal_name,
               joint_state_topic_name);
    return Value ();
  }
} // namespace ros_state_publish
} // namespace command
} // namespace dynamicgraph
