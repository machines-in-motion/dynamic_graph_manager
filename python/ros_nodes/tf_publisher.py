#!/usr/bin/env python

"""@package dynamic_graph_manager

@file __init__.py
@author Maximilien Naveau (maximilien.naveau@gmail.com)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellshaft.
@date 2019-05-22

@brief This script looks for a particular tf transformation
and publish it as a TransformStamped topic.
This may be useful to insert tf frames into dynamic-graph
through dynamic_graph_bridge.

"""


import rospy

import tf
import geometry_msgs.msg


def main():
    rospy.init_node('tf_publisher', anonymous=True)

    frame = rospy.get_param('~frame', '')
    childFrame = rospy.get_param('~child_frame', '')
    topic = rospy.get_param('~topic', '')
    rateSeconds = rospy.get_param('~rate', 5)

    if not frame or not childFrame or not topic:
        logpy.error("frame, childFrame and topic are required parameters")
        return

    rate = rospy.Rate(rateSeconds)
    tl = tf.TransformListener()
    pub = rospy.Publisher(topic, geometry_msgs.msg.TransformStamped)

    transform = geometry_msgs.msg.TransformStamped()
    transform.header.frame_id = frame
    transform.child_frame_id = childFrame

    ok = False
    while not rospy.is_shutdown() and not ok:
        try:
            tl.waitForTransform(childFrame, frame,
                                rospy.Time(), rospy.Duration(0.1))
            ok = True
        except tf.Exception, e:
            rospy.logwarn("waiting for tf transform")
            ok = False

    while not rospy.is_shutdown():
        time = tl.getLatestCommonTime(frame, childFrame)
        (p, q) = tl.lookupTransform(childFrame, frame, time)
        transform.header.seq += 1
        transform.header.stamp = time

        transform.transform.translation.x = p[0]
        transform.transform.translation.y = p[1]
        transform.transform.translation.z = p[2]

        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        pub.publish(transform)
        rate.sleep()

main()
