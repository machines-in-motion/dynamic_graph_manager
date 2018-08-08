/**
 * \file ros_init.cpp
 * \brief a ROS singleton
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file implements a ROS global singleton object. It is used to initialize
 * ROS and get a node handle specific to the DynamicGraphManager class.
 */

#include <dynamic_graph_manager/ros_init.hh>

namespace dynamic_graph
{
  /**
   * @brief GLOBAL_ROS_VAR is global variable that acts as a singleton on the
   * ROS node handle and the spinner.
   *
   * The use of the std::unique_ptr allows to delete the object and re-create
   * at will. It is usefull to reset the ros environment specifically for
   * unittesting.
   */
  static std::unique_ptr<GlobalRos> GLOBAL_ROS_VAR(nullptr);

  ros::NodeHandle& ros_init ()
  {
    if (GLOBAL_ROS_VAR == nullptr)
    {
      GLOBAL_ROS_VAR.reset(new GlobalRos());
    }
    /** If the node handle does not exist we call the global method ros::init.
     * This method has for purpose to initialize the ROS environment. The
     * creation of ROS object is permitted only after the call of this function.
     * After ros::init being called we create the node hanlde which allows in
     * turn to advertize the ROS services, or create topics (data pipe).
     */
    if (!GLOBAL_ROS_VAR->node_handle_)
    {
      /** call ros::init */
      int argc = 1;
      char* arg0 = strdup("dynamic_graph_manager");
      char* argv[] = {arg0, nullptr};
      ros::init(argc, argv, "dynamic_graph_manager");
      free (arg0);

      /** ros::NodeHandle instanciation */
      GLOBAL_ROS_VAR->node_handle_ = boost::make_shared<ros::NodeHandle>
                                    ("dynamic_graph_manager");
    }
    /** If spinner is not created we create it. Here we can safely assume that
     * ros::init was called before.
     *
     */
    if (!GLOBAL_ROS_VAR->async_spinner_)
    {
      /** create the spinner */
      GLOBAL_ROS_VAR->async_spinner_ = boost::make_shared<ros::AsyncSpinner> (4);
      /** run the spinner in a different thread */
      GLOBAL_ROS_VAR->async_spinner_->start ();
    }
    /** Return a reference to the node handle so any function can use it */
    return *GLOBAL_ROS_VAR->node_handle_;
  }

  ros::AsyncSpinner& ros_spinner ()
  {
    if (!GLOBAL_ROS_VAR->async_spinner_)
    {
      dynamic_graph::ros_init();
    }
    return *GLOBAL_ROS_VAR->async_spinner_;
  }

  void ros_shutdown ()
  {
    GLOBAL_ROS_VAR.reset(nullptr);
    ros::shutdown();
  }
} // end of namespace dynamic_graph.
