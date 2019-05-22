/**
 * @file ros_queued_subscribe.hxx
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 * @date 2019-05-22
 */

#ifndef DYNAMIC_GRAPH_ROS_QUEUED_SUBSCRIBE_HXX
# define DYNAMIC_GRAPH_ROS_QUEUED_SUBSCRIBE_HXX
# include <vector>
# include <boost/bind.hpp>
# include <boost/date_time/posix_time/posix_time.hpp>
# include <dynamic-graph/signal-caster.h>
# include <dynamic-graph/linear-algebra.h>
# include <dynamic-graph/signal-cast-helper.h>
# include <std_msgs/Float64.h>
# include "dynamic_graph_manager/Matrix.h"
# include "dynamic_graph_manager/Vector.h"

namespace dg = dynamicgraph;
typedef boost::mutex::scoped_lock scoped_lock;

namespace dynamic_graph
{
  namespace internal
  {
    static const int BUFFER_SIZE = 1 << 10;

    template <typename T>
    struct Add
    {
      void operator () (RosQueuedSubscribe& rosSubscribe,
			const std::string& type,
			const std::string& signal,
			const std::string& topic)
      {
        typedef typename DgToRos<T>::dg_t dg_t;
        typedef typename DgToRos<T>::ros_const_ptr_t ros_const_ptr_t;
        typedef BindedSignal<dg_t, BUFFER_SIZE> BindedSignal_t;
	typedef typename BindedSignal_t::Signal_t Signal_t;

	// Initialize the bindedSignal object.
        BindedSignal_t* bs = new BindedSignal_t(&rosSubscribe);
        DgToRos<T>::setDefault (bs->last);

	// Initialize the signal.
	boost::format signalName ("RosQueuedSubscribe(%1%)::output(%2%)::%3%");
	signalName % rosSubscribe.getName () % type % signal;

	bs->signal.reset (new Signal_t (signalName.str ()));
        bs->signal->setFunction (boost::bind(&BindedSignal_t::reader, bs, _1, _2));
	rosSubscribe.signalRegistration (*bs->signal);

	// Initialize the subscriber.
	typedef boost::function<void (const ros_const_ptr_t& data)> callback_t;
	callback_t callback = boost::bind
	  (&BindedSignal_t::template writer<ros_const_ptr_t>, bs, _1);

  // Keep 50 messages in queue, but only 20 are sent every 100ms
  // -> No message should be lost because of a full buffer
	bs->subscriber =
	  boost::make_shared<ros::Subscriber>
	  (rosSubscribe.nh ().subscribe (topic, BUFFER_SIZE, callback)); 

	RosQueuedSubscribe::bindedSignal_t bindedSignal (bs);
	rosSubscribe.bindedSignal ()[signal] = bindedSignal;
      }
    };

    // template <typename T, typename R>
    template <typename T, int N>
    template <typename R>
    void BindedSignal<T, N>::writer (const R& data)
    {
      // synchronize with method clear
      boost::mutex::scoped_lock lock(wmutex);
      if (full()) {
        rmutex.lock();
        frontIdx = (frontIdx + 1) % N;
        rmutex.unlock();
      }
      converter (buffer[backIdx], data);
      // No need to synchronize with reader here because:
      // - if the buffer was not empty, then it stays not empty,
      // - if it was empty, then the current value will be used at next time. It
      //   means the transmission bandwidth is too low.
      if (!init) {
        last = buffer[backIdx];
        init = true;
      }
      backIdx = (backIdx+1) % N;
    }

    template <typename T, int N>
    T& BindedSignal<T, N>::reader (T& data, int time)
    {
      // synchronize with method clear:
      // If reading from the list cannot be done, then return last value.
      scoped_lock lock(rmutex, boost::try_to_lock);
      if (!lock.owns_lock() || entity->readQueue() == -1 || time < entity->readQueue()) {
        data = last;
      } else {
        if (empty())
          data = last;
        else {
          data = buffer[frontIdx];
          frontIdx = (frontIdx + 1) % N;
          last = data;
        }
      }
      return data;
    }
  } // end of namespace internal.

  template <typename T>
  void RosQueuedSubscribe::add (const std::string& type, const std::string& signal, const std::string& topic)
  {
    internal::Add<T> () (*this, type, signal, topic);
  }
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_QUEUED_SUBSCRIBE_HXX
