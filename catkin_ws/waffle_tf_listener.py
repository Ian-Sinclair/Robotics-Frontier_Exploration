#!/usr/bin/env python
import rospy
import tf2_ros

'''
    Transformation listener to record the robots current position
    with respect to the map frame. (This will be the robots (base_footprint frame's)
    coordinate location on the map.)
'''


if __name__ == '__main__' :
    rospy.init_node('waffle_tf_listener')

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

