/**
 * @file ros_subscribe.hxx
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-05-22
 */

#ifndef DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
# define DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
# include <vector>
# include <boost/bind.hpp>
# include <boost/date_time/posix_time/posix_time.hpp>
# include <dynamic-graph/signal-caster.h>
# include <dynamic-graph/linear-algebra.h>
# include <dynamic-graph/signal-cast-helper.h>
# include <std_msgs/Float64.h>
# include "dynamic_graph_manager/Matrix.h"
# include "dynamic_graph_manager/Vector.h"
# include "ros_time.hh"

namespace dg = dynamicgraph;

namespace dynamic_graph
{
  template <typename R, typename S>
  void
  RosSubscribe::callback
  (boost::shared_ptr<dynamicgraph::SignalPtr<S, int> > signal,
   const R& data)
  {
    typedef S dg_t;
    dg_t value;
    converter (value, data);
    signal->setConstant (value);
  }

  template <typename R>
  void
  RosSubscribe::callbackTimestamp
  (boost::shared_ptr<dynamicgraph::SignalPtr<ptime, int> > signal,
   const R& data)
  {
    ptime time =
      rosTimeToPtime (data->header.stamp);
    signal->setConstant(time);
  }

  namespace internal
  {
    template <typename T>
    struct Add
    {
      void operator () (RosSubscribe& RosSubscribe,
			const std::string& signal,
			const std::string& topic)
      {
        typedef typename DgToRos<T>::dg_t dg_t;
        typedef typename DgToRos<T>::ros_const_ptr_t ros_const_ptr_t;
        typedef typename DgToRos<T>::signalIn_t signal_t;

	// Initialize the bindedSignal object.
	RosSubscribe::bindedSignal_t bindedSignal;

	// Initialize the signal.
	boost::format signalName ("RosSubscribe(%1%)::%2%");
	signalName % RosSubscribe.getName () % signal;

	boost::shared_ptr<signal_t> signal_ (new signal_t (0, signalName.str ()));
  DgToRos<T>::setDefault(*signal_);
	bindedSignal.first = signal_;
	RosSubscribe.signalRegistration (*bindedSignal.first);

	// Initialize the subscriber.
	typedef boost::function<void (const ros_const_ptr_t& data)> callback_t;
	callback_t callback = boost::bind
	  (&RosSubscribe::callback<ros_const_ptr_t, dg_t>,
	   &RosSubscribe, signal_, _1);

	bindedSignal.second =
	  boost::make_shared<ros::Subscriber>
	  (RosSubscribe.nh ().subscribe (topic, 1, callback));

	RosSubscribe.bindedSignal ()[signal] = bindedSignal;
      }
    };

    template <typename T>
    struct Add<std::pair<T, dg::Vector> >
    {
      void operator () (RosSubscribe& RosSubscribe,
			const std::string& signal,
			const std::string& topic)
      {
	typedef std::pair<T, dg::Vector> type_t;

        typedef typename DgToRos<type_t>::dg_t dg_t;
        typedef typename DgToRos<type_t>::ros_const_ptr_t ros_const_ptr_t;
        typedef typename DgToRos<type_t>::signalIn_t signal_t;

	// Initialize the bindedSignal object.
	RosSubscribe::bindedSignal_t bindedSignal;

	// Initialize the signal.
	boost::format signalName ("RosSubscribe(%1%)::%2%");
	signalName % RosSubscribe.getName () % signal;

	boost::shared_ptr<signal_t> signal_
	  (new signal_t (0, signalName.str ()));
        DgToRos<T>::setDefault(*signal_);
	bindedSignal.first = signal_;
	RosSubscribe.signalRegistration (*bindedSignal.first);

	// Initialize the publisher.
	typedef boost::function<void (const ros_const_ptr_t& data)> callback_t;
	callback_t callback = boost::bind
	  (&RosSubscribe::callback<ros_const_ptr_t, dg_t>,
	   &RosSubscribe, signal_, _1);

	bindedSignal.second =
	  boost::make_shared<ros::Subscriber>
	  (RosSubscribe.nh ().subscribe (topic, 1, callback));

	RosSubscribe.bindedSignal ()[signal] = bindedSignal;


	// Timestamp.
	typedef dynamicgraph::SignalPtr<RosSubscribe::ptime, int>
	  signalTimestamp_t;
	std::string signalTimestamp =
	  (boost::format ("%1%%2%") % signal % "Timestamp").str ();

	// Initialize the bindedSignal object.
	RosSubscribe::bindedSignal_t bindedSignalTimestamp;

	// Initialize the signal.
	boost::format signalNameTimestamp ("RosSubscribe(%1%)::%2%");
	signalNameTimestamp % RosSubscribe.name % signalTimestamp;

	boost::shared_ptr<signalTimestamp_t> signalTimestamp_
	  (new signalTimestamp_t (0, signalNameTimestamp.str ()));

	RosSubscribe::ptime zero (rosTimeToPtime (ros::Time (0, 0)));
	signalTimestamp_->setConstant (zero);
	bindedSignalTimestamp.first = signalTimestamp_;
	RosSubscribe.signalRegistration (*bindedSignalTimestamp.first);

	// Initialize the publisher.
	typedef boost::function<void (const ros_const_ptr_t& data)> callback_t;
	callback_t callbackTimestamp = boost::bind
	  (&RosSubscribe::callbackTimestamp<ros_const_ptr_t>,
	   &RosSubscribe, signalTimestamp_, _1);

	bindedSignalTimestamp.second =
	  boost::make_shared<ros::Subscriber>
	  (RosSubscribe.nh ().subscribe (topic, 1, callbackTimestamp));

	RosSubscribe.bindedSignal ()[signalTimestamp] = bindedSignalTimestamp;
      }
    };
  } // end of namespace internal.

  template <typename T>
  void RosSubscribe::add (const std::string& signal, const std::string& topic)
  {
    internal::Add<T> () (*this, signal, topic);
  }
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
