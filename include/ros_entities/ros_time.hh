/**
 * @file ros_time.hh
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-05-22
 */

#ifndef DYNAMIC_GRAPH_ROS_TIME_HH
# define DYNAMIC_GRAPH_ROS_TIME_HH

# include <ros/time.h>
# include <boost/date_time/posix_time/posix_time_types.hpp>
# include <dynamic-graph/signal.h>
# include <dynamic-graph/entity.h>

namespace dynamic_graph {

  class RosTime : public dynamicgraph::Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL ();
  public:
    dynamicgraph::Signal <boost::posix_time::ptime, int> now_;
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
