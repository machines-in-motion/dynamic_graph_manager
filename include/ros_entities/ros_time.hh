///
/// Copyright 2012 CNRS
///
/// Author: Florent Lamiraux

#ifndef DYNAMIC_GRAPH_ROS_TIME_HH
# define DYNAMIC_GRAPH_ROS_TIME_HH

# include <ros/time.h>
# include <boost/date_time/posix_time/posix_time_types.hpp>
# include <dynamic-graph/signal.h>
# include <dynamic-graph/entity.h>

namespace dynamicgraph {

  class RosTime : public dynamicgraph::Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL ();
  public:
    Signal <boost::posix_time::ptime, int> now_;
    RosTime (const std::string& name);
    virtual std::string getDocString () const;
  protected:
    boost::posix_time::ptime&
    update (boost::posix_time::ptime& time, const int& t);
  private:
    static const std::string docstring_;
  }; // class RosTime

} // namespace dynamicgraph

#endif // DYNAMIC_GRAPH_ROS_TIME_HH
