#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class Markers(object):

    def __init__(self):
        self.marker_publisher = rospy.Publisher(
            'marker_visual', Marker, queue_size=1)
        self.marker_subscriber = rospy.Subscriber("/robot_0/base_scan", LaserScan)
        self.rate = rospy.Rate(1)
        self.init_marker(index=0)


    def init_marker(self, index=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = '/base_link'
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = 'robot_0_visual'
        self.marker_object.id = index
        self.marker_object.type = Marker.LINE_STRIP
        self.marker_object.action = Marker.ADD

        '''my_point = Point()
        my_point.z = z_val
        self.marker_object.pose.position = my_point'''

        self.marker_object.pose.orientation.x = 0.0
        self.marker_object.pose.orientation.y = 0.0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale.y = 0.02
        self.marker_object.scale.x = 0.02
        self.marker_object.scale.z = 0.02

        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 1.0
        self.marker_object.color.a = 1.0

        self.marker_object.points = []

        first_point = Point()
        first_point.x = -8.0
        first_point.y = -2.0
        first_point.z = 0.0
        self.marker_object.points.append(first_point)

        second_point = Point()
        second_point.x = 4.5
        second_point.y = 9.0
        second_point.z = 0.0
        self.marker_object.points.append(second_point)


        self.marker_object.lifetime = rospy.Duration(0)

    def start(self):
        while not rospy.is_shutdown():
            self.marker_publisher.publish(self.marker_object)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node("marker_ransac_node", anonymous=True)
    m_object = Markers()
    try:
        m_object.start()
    except rospy.ROSInterruptException:
        pass
