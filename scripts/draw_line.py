#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String

def talker():


    # test_pub = rospy.Publisher('test', Marker, queue_size=10)
    rospy.init_node('line_pub_example', anonymous=True)
    pub_line_min_dist = rospy.Publisher('line_min_dist', Marker, queue_size=10)
    rospy.loginfo('Publishing example line')
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []
        # first point
        first_line_point = Point()
        first_line_point.x = -1.325
        first_line_point.y = 6.5
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        # second point
        second_line_point = Point()
        second_line_point.x = -4.192
        second_line_point.y = 6.5
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        # Publish the Marker
        pub_line_min_dist.publish(marker)
        r.sleep()

if __name__  == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass