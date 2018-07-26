#include <dynamic_graph_manager/ros_init.hh>

namespace dynamic_graph
{
  /**
   * @brief ros is global variable that acts as a singleton on the ROS node
   * handle and the spinner.
   */
  GlobalRos GLOBAL_ROS_VAR;

  ros::NodeHandle& ros_init ()
  {
    /** If the node handle does not exist we call the global method ros::init.
     * This method has for purpose to initialize the ROS environment. The
     * creation of ROS object is permitted only after the call of this function.
     * After ros::init being called we create the node hanlde which allows in
     * turn to advertize the ROS services, or create topics (data pipe).
     */
    if (!GLOBAL_ROS_VAR.node_handle_)
    {
      /** call ros::init */
      int argc = 1;
      char* arg0 = strdup("dynamic_graph_manager");
      char* argv[] = {arg0, 0};
      ros::init(argc, argv, "dynamic_graph_manager");
      free (arg0);

      /** ros::NodeHandle instanciation */
      GLOBAL_ROS_VAR.node_handle_ = boost::make_shared<ros::NodeHandle>
                                    ("dynamic_graph_manager");
    }
    /** If spinner is not created we create it. Here we can safely assume that
     * ros::init was called before.
     *
     */
    if (!GLOBAL_ROS_VAR.async_spinner_)
    {
      /** create the spinner */
      GLOBAL_ROS_VAR.async_spinner_ = boost::make_shared<ros::AsyncSpinner> (4);
      /** run the spinner in a different thread */
      GLOBAL_ROS_VAR.async_spinner_->start ();
    }
    /** Return a reference to the node handle so any function can use it */
    return *GLOBAL_ROS_VAR.node_handle_;
  }

  ros::AsyncSpinner& ros_spinner ()
  {
    if (!GLOBAL_ROS_VAR.async_spinner_)
    {
      dynamic_graph::ros_init();
    }
    return *GLOBAL_ROS_VAR.async_spinner_;
  }
} // end of namespace dynamic_graph.
