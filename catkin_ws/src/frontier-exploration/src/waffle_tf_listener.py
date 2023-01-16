#!/usr/bin/env python
import rospy
import tf2_ros

if __name__ == '__main__' :
    rospy.init_node('waffle_tf_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    #turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown() :
        try:
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
            rospy.loginfo(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            rospy.logerr('ERROR: Could not find map to base_footprint transform')
            continue

       # msg = geometry_msgs.msg.Twist()

       # msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
       # msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

    #    turtle_vel.publish(msg)

        rate.sleep()
