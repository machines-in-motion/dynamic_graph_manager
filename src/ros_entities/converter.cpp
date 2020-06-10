/**
 * @file converter.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <ros_entities/converter.hh>

namespace dynamic_graph
{
boost::posix_time::ptime rosTimeToPtime(const ros::Time& rosTime)
{
    ptime time(date(1970, 1, 1),
               seconds(rosTime.sec) + microseconds(rosTime.nsec / 1000));
    return time;
}

ros::Time pTimeToRostime(const boost::posix_time::ptime& time)
{
    static ptime timeStart(date(1970, 1, 1));
    time_duration diff = time - timeStart;

    uint32_t sec =
        static_cast<unsigned int>(diff.ticks()) /
        static_cast<unsigned int>(time_duration::rep_type::res_adjust());
    uint32_t nsec = static_cast<unsigned int>(diff.fractional_seconds());

    return ros::Time(sec, nsec);
}

}  // namespace dynamic_graph
