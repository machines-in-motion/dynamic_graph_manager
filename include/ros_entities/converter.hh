/**
 * @file converter.hh
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
 * @date 2019-05-22
 */
#ifndef DYNAMIC_GRAPH_ROS_CONVERTER_HH
# define DYNAMIC_GRAPH_ROS_CONVERTER_HH

# include <stdexcept>
# include "dg_to_ros.hh"

# include <ros/time.h>
# include <std_msgs/Header.h>

// not used?
//# include <LinearMath/btMatrix3x3.h>
//# include <LinearMath/btQuaternion.h>

#ifdef MAC_OS
// On MAC_OS a macro "tolower" when __APPLE__ is defined.
// For compatibility reason we need to undefine it as it
// interfere with the std::tolower function used in boost/date_time
#    undef tolower
#endif // MAC_OS

# include <boost/static_assert.hpp>
# include <boost/date_time/date.hpp>
# include <boost/date_time/posix_time/posix_time.hpp>

# define DG_TO_ROS_IMPL(T)						\
  template <>								\
  inline void								\
  converter (DgToRos<T>::ros_t& dst, const DgToRos<T>::dg_t& src)

# define ROS_TO_DG_IMPL(T)						\
  template <>								\
  inline void								\
  converter (DgToRos<T>::dg_t& dst, const DgToRos<T>::ros_t& src)

namespace dynamic_graph
{
  inline
  void
  makeHeader(std_msgs::Header& header)
  {
    header.seq = 0;
    header.stamp = ros::Time::now ();
    header.frame_id = "/dynamic_graph/world";
  }

  /// \brief Handle ROS <-> dynamic-graph conversion.
  ///
  /// Implements all ROS/dynamic-graph conversions required by the
  /// bridge.
  ///
  ///Relies on the DgToRos type trait to make sure valid pair of
  /// types are used.
  template <typename D, typename S>
  void converter (D& dst, const S& src);

  // Double
  DG_TO_ROS_IMPL(double)
  {
    dst.data = src;
  }

  ROS_TO_DG_IMPL(double)
  {
    dst = src.data;
  }

  // Unsigned
  DG_TO_ROS_IMPL(unsigned int)
  {
    dst.data = src;
  }

  ROS_TO_DG_IMPL(unsigned int)
  {
    dst = src.data;
  }

  // Vector
  DG_TO_ROS_IMPL(Vector)
  {
    dst.data.resize (src.size ());
    for (int i = 0; i < src.size (); ++i)
      dst.data[i] =  src (i);
  }

  ROS_TO_DG_IMPL(Vector)
  {
    dst.resize (src.data.size ());
    for (unsigned int i = 0; i < src.data.size (); ++i)
      dst (i) =  src.data[i];
  }

  // Vector3
  DG_TO_ROS_IMPL(specific::Vector3)
  {
    if (src.size () > 0)
      {
	dst.x =  src (0);
	if (src.size () > 1)
	  {
	    dst.y =  src (1);
	    if (src.size () > 2)
	      dst.z =  src (2);
	  }
      }
  }

  ROS_TO_DG_IMPL(specific::Vector3)
  {
    dst.resize (3);
    dst (0) =  src.x;
    dst (1) =  src.y;
    dst (2) =  src.z;
  }

  // Matrix
  DG_TO_ROS_IMPL(Matrix)
  {

    //TODO: Confirm Ros Matrix Storage order. It changes the RosMatrix to ColMajor.
    dst.width = (unsigned int)src.rows ();
    dst.data.resize (src.cols () * src.rows ());
    for (int i = 0; i < src.cols () * src.rows (); ++i)
      dst.data[i] =  src.data()[i];
  }
  
  ROS_TO_DG_IMPL(Matrix)
  {
    dst.resize (src.width, (unsigned int) src.data.size () / 
		(unsigned int)src.width);
    for (unsigned i = 0; i < src.data.size (); ++i)
      dst.data()[i] =  src.data[i];
  }

  // Homogeneous matrix.
  DG_TO_ROS_IMPL(MatrixHomogeneous)
  {

    VectorQuaternion q(src.linear());
    dst.translation.x = src.translation()(0);
    dst.translation.y = src.translation()(1);
    dst.translation.z = src.translation()(2);

    dst.rotation.x = q.x();
    dst.rotation.y = q.y();
    dst.rotation.z = q.z();
    dst.rotation.w = q.w();

  }

  ROS_TO_DG_IMPL(MatrixHomogeneous)
  {
    VectorQuaternion q(src.rotation.w,
			    src.rotation.x,
			    src.rotation.y,
			    src.rotation.z);
    dst.linear() = q.matrix();

    // Copy the translation component.
    dst.translation()(0) = src.translation.x;
    dst.translation()(1) = src.translation.y;
    dst.translation()(2) = src.translation.z;
  }


  // Twist.
  DG_TO_ROS_IMPL(specific::Twist)
  {
    if (src.size () != 6)
      throw std::runtime_error ("failed to convert invalid twist");
    dst.linear.x = src (0);
    dst.linear.y = src (1);
    dst.linear.z = src (2);
    dst.angular.x = src (3);
    dst.angular.y = src (4);
    dst.angular.z = src (5);
  }

  ROS_TO_DG_IMPL(specific::Twist)
  {
    dst.resize (6);
    dst (0) = src.linear.x;
    dst (1) = src.linear.y;
    dst (2) = src.linear.z;
    dst (3) = src.angular.x;
    dst (4) = src.angular.y;
    dst (5) = src.angular.z;
  }


  /// \brief This macro generates a converter for a stamped type from
  /// dynamic-graph to ROS.  I.e. A data associated with its
  /// timestamp.
# define DG_BRIDGE_TO_ROS_MAKE_STAMPED_IMPL(T, ATTRIBUTE, EXTRA)	\
  template <>								\
  inline void converter							\
  (DgToRos<std::pair<T, Vector> >::ros_t& dst,			\
   const DgToRos<std::pair<T, Vector> >::dg_t& src)		\
  {									\
    makeHeader(dst.header);						\
    converter<DgToRos<T>::ros_t, DgToRos<T>::dg_t> (dst.ATTRIBUTE, src); \
    do { EXTRA } while (0);						\
  }									\
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  DG_BRIDGE_TO_ROS_MAKE_STAMPED_IMPL(specific::Vector3, vector, ;);
  DG_BRIDGE_TO_ROS_MAKE_STAMPED_IMPL(MatrixHomogeneous, transform,
				     dst.child_frame_id = "";);
  DG_BRIDGE_TO_ROS_MAKE_STAMPED_IMPL(specific::Twist, twist, ;);


  /// \brief This macro generates a converter for a shared pointer on
  ///        a ROS type to a dynamic-graph type.
  ///
  ///        A converter for the underlying type is required.  I.e. to
  ///        convert a shared_ptr<T> to T', a converter from T to T'
  ///        is required.
# define DG_BRIDGE_MAKE_SHPTR_IMPL(T)					\
  template <>								\
  inline void converter							\
  (DgToRos<T>::dg_t& dst,						\
   const boost::shared_ptr<DgToRos<T>::ros_t const>& src)		\
  {									\
    converter<DgToRos<T>::dg_t, DgToRos<T>::ros_t> (dst, *src);	\
  }									\
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  DG_BRIDGE_MAKE_SHPTR_IMPL(double);
  DG_BRIDGE_MAKE_SHPTR_IMPL(unsigned int);
  DG_BRIDGE_MAKE_SHPTR_IMPL(Vector);
  DG_BRIDGE_MAKE_SHPTR_IMPL(specific::Vector3);
  DG_BRIDGE_MAKE_SHPTR_IMPL(Matrix);
  DG_BRIDGE_MAKE_SHPTR_IMPL(MatrixHomogeneous);
  DG_BRIDGE_MAKE_SHPTR_IMPL(specific::Twist);

  /// \brief This macro generates a converter for a stamped type.
  /// I.e. A data associated with its timestamp.
  ///
  /// FIXME: the timestamp is not yet forwarded to the dg signal.
# define DG_BRIDGE_MAKE_STAMPED_IMPL(T, ATTRIBUTE, EXTRA)		\
  template <>								\
  inline void converter							\
  (DgToRos<std::pair<T, Vector> >::dg_t& dst,			\
   const DgToRos<std::pair<T, Vector> >::ros_t& src)		\
  {									\
    converter<DgToRos<T>::dg_t, DgToRos<T>::ros_t> (dst, src.ATTRIBUTE); \
    do { EXTRA } while (0);						\
  }									\
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  DG_BRIDGE_MAKE_STAMPED_IMPL(specific::Vector3, vector, ;);
  DG_BRIDGE_MAKE_STAMPED_IMPL(MatrixHomogeneous, transform, ;);
  DG_BRIDGE_MAKE_STAMPED_IMPL(specific::Twist, twist, ;);

  /// \brief This macro generates a converter for a shared pointer on
  /// a stamped type.  I.e. A data associated with its timestamp.
  ///
  /// FIXME: the timestamp is not yet forwarded to the dg signal.
# define DG_BRIDGE_MAKE_STAMPED_SHPTR_IMPL(T, ATTRIBUTE, EXTRA)		\
  template <>								\
  inline void converter							\
  (DgToRos<std::pair<T, Vector> >::dg_t& dst,			\
   const boost::shared_ptr						\
   <DgToRos<std::pair<T, Vector> >::ros_t const>& src)		\
  {									\
    converter<DgToRos<T>::dg_t, DgToRos<T>::ros_t> (dst, src->ATTRIBUTE); \
    do { EXTRA } while (0);						\
  }									\
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  DG_BRIDGE_MAKE_STAMPED_SHPTR_IMPL(specific::Vector3, vector, ;);
  DG_BRIDGE_MAKE_STAMPED_SHPTR_IMPL(MatrixHomogeneous, transform, ;);
  DG_BRIDGE_MAKE_STAMPED_SHPTR_IMPL(specific::Twist, twist, ;);


  /// \brief If an impossible/unimplemented conversion is required, fail.
  ///
  /// IMPORTANT, READ ME:
  ///
  /// If the compiler generates an error in the following function,
  /// this is /normal/.
  ///
  /// This error means that either you try to use an undefined
  /// conversion.  You can either fix your code or provide the wanted
  /// conversion by updating this header.
  template <typename U, typename V>
  inline void converter (U& dst, V& src)
  {
    // This will always fail if instantiated.
    BOOST_STATIC_ASSERT (sizeof (U) == 0);
  }

  typedef boost::posix_time::ptime ptime;
  typedef boost::posix_time::seconds seconds;
  typedef boost::posix_time::microseconds microseconds;
  typedef boost::posix_time::time_duration time_duration;
  typedef boost::gregorian::date date;

  boost::posix_time::ptime rosTimeToPtime (const ros::Time& rosTime);

  ros::Time pTimeToRostime (const boost::posix_time::ptime& time);
} // end of namespace dynamic_graph.

#endif //! DYNAMIC_GRAPH_ROS_CONVERTER_HH
