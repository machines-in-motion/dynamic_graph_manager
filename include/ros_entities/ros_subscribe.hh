/**
 * @file ros_subscribe.hh
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-05-22
 */

#ifndef DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
# define DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
# include <iostream>
# include <map>

# include <boost/shared_ptr.hpp>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-time-dependent.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/command.h>

# include <ros_entities/matrix_geometry.hh>

# include <ros/ros.h>

# include "converter.hh"
# include "dg_to_ros.hh"

namespace dynamic_graph
{
  class RosSubscribe;

  namespace command
  {
    namespace rosSubscribe
    {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;

# define ROS_SUBSCRIBE_MAKE_COMMAND(CMD)			\
      class CMD : public Command			\
      {							\
      public:						\
	CMD (RosSubscribe& entity,				\
	     const std::string& docstring);		\
	virtual Value doExecute ();			\
      }

      ROS_SUBSCRIBE_MAKE_COMMAND(Add);
      ROS_SUBSCRIBE_MAKE_COMMAND(Clear);
      ROS_SUBSCRIBE_MAKE_COMMAND(List);
      ROS_SUBSCRIBE_MAKE_COMMAND(Rm);

#undef ROS_SUBSCRIBE_MAKE_COMMAND

    } // end of namespace errorEstimator.
  } // end of namespace command.


  namespace internal
  {
    template <typename T>
    struct Add;
  } // end of internal namespace.

  /// \brief Publish ROS information in the dynamic-graph.
  class RosSubscribe : public dynamicgraph::Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
    typedef boost::posix_time::ptime ptime;
  public:
    typedef std::pair<boost::shared_ptr<dynamicgraph::SignalBase<int> >,
		      boost::shared_ptr<ros::Subscriber> >
      bindedSignal_t;

    RosSubscribe (const std::string& n);
    virtual ~RosSubscribe ();

    virtual std::string getDocString () const;
    void display (std::ostream& os) const;

    void add (const std::string& signal, const std::string& topic);
    void rm (const std::string& signal);
    std::string list ();
    void clear ();

    template <typename T>
    void add (const std::string& signal, const std::string& topic);

    std::map<std::string, bindedSignal_t>&
    bindedSignal ()
    {
      return bindedSignal_;
    }

    ros::NodeHandle& nh ()
    {
      return nh_;
    }

    template <typename R, typename S>
    void callback
    (boost::shared_ptr<dynamicgraph::SignalPtr<S, int> > signal,
     const R& data);

    template <typename R>
    void callbackTimestamp
    (boost::shared_ptr<dynamicgraph::SignalPtr<ptime, int> > signal,
     const R& data);

    template <typename T>
    friend class internal::Add;
  private:
    static const std::string docstring_;
    ros::NodeHandle& nh_;
    std::map<std::string, bindedSignal_t> bindedSignal_;
  };
} // end of namespace dynamicgraph.

# include "ros_subscribe.hxx"
#endif //! DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
