#!/usr/bin/env python

import tf2_ros
import rospy


class navigation() :
    def __init__() :
        pass

    def init_node() :
        rospy.init_node('Frontier Navigation')
    
    def init_subscriber() :
        pass

    def init_listener() :
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() :
            try:
                trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
                rospy.loginfo(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                rospy.logerr('ERROR: Could not find map to base_footprint transform')
                continue
            rate.sleep()
        