#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from util import point
from geometry_msgs.msg import Pose, PoseArray


class RviZ_publisher() :
    def __init__(self) :
        self.init_node()
        self.init_publisher()
        self.init_subscriber()
    
    def init_node(self):
        rospy.init_node('mapMarkerPub')
    
    def init_publisher(self) :
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

    def init_subscriber(self) :
        self.point_Sub = rospy.Subscriber('points', PoseArray, self.points_callback)
        rospy.spin()
    
    def points_callback(self, data) :
        points = [point(p.position.x,p.position.y,0) for p in data.poses ]
        rospy.loginfo(len(points))
        self.pub_point(points, r = 1)


    def pub_point( self , points, w=1 , nx=0 , ny=0 , nz=0 , r=1.0, g=0.0, b=0.0, alpha=1.0, id=0, namespace='marker' ) :
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        # marker type = 8 should be a point, refer to the msg/Marker config for details.
        marker.ns = namespace
        marker.type = 8
        marker.id = id

        marker.pose.orientation.x = nx
        marker.pose.orientation.y = ny
        marker.pose.orientation.z = nz
        marker.pose.orientation.w = w

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        # Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = float(alpha)

        if points == None :
            rospy.logerr('list of points need not be none in RviZ Publisher -> pub_point')
            return None

        marker.points = points


        marker.lifetime = rospy.Duration()

        rospy.loginfo('publishing....')
        self.marker_pub.publish(marker)
        rospy.rostime.wallsleep(1.0)





if __name__ == '__main__' :
    pubs = RviZ_publisher()

