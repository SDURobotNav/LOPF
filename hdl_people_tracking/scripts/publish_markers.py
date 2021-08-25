#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray, Marker

if __name__ =="__main__":
    rospy.init_node("publish_markers")
    pub_markers = rospy.Publisher("/Markers",MarkerArray,queue_size=10)
    pub_marker = rospy.Publisher("/Marker", Marker, queue_size=10)
    markers = MarkerArray()

    a = 0
    b = 0
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "map"
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    while not rospy.is_shutdown():

        # marker.lifetime = rospy.Duration(1)
        marker.id = a
        marker.pose.position.x = b
        marker.pose.position.y = a
        marker.pose.position.z = 2
        marker.pose.orientation.w = 1.0

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.4

        marker.scale.x = 2
        marker.scale.y = 1
        marker.scale.z = 0.2
        a+=1
        b+=1
        pub_marker.publish(marker)
        markers.markers.append(marker)
        pub_markers.publish(markers)
        rospy.sleep(1.0)
        print ("hhhhh")