#!/usr/bin/env python
import math
import random
from array import array
from turtle import Turtle

import rospy
import tf
import turtlesim.msg
import turtlesim.srv
from geometry_msgs.msg import Point, Twist
from laser_geometry import LaserProjection
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

is_ransac_executed = True
rvizpub = rospy.Publisher("marker", Marker, queue_size=10)

def slope(dx, dy):
    return (dy / dx) if dx else None
#def get_line_equation(point1,point2)


class Point_info:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return '({}, {})'.format(self.x, self.y)

    def __repr__(self):
        return 'Point_info({}, {})'.format(self.x, self.y)

    def halfway(self, target):
        midx = (self.x + target.x) / 2
        midy = (self.y + target.y) / 2
        return Point_info(midx, midy)

    def distance(self, target):
        dx = target.x - self.x
        dy = target.y - self.y
        return (dx*dx + dy*dy) ** 0.5

    def reflect_x(self):
        return Point_info(-self.x,self.y)

    def reflect_y(self):
        return Point_info(self.x,-self.y)

    def reflect_x_y(self):
        return Point_info(-self.x, -self.y)

    def slope_from_origin(self):
        return slope(self.x, self.y)

    def slope(self, target):
        return slope(target.x - self.x, target.y - self.y)

    def y_int(self, target):       # <= here's the magic
        return self.y - self.slope(target)*self.x

    def line_equation(self, target):
        slope = self.slope(target)

        y_int = self.y_int(target)
        
        return (slope,y_int)
        return 'y = {}x {} {}'.format(slope, y_int)

    def line_function(self, target):
        slope = self.slope(target)
        y_int = self.y_int(target)
        def fn(x):
            return slope*x + y_int
        return fn

    def distance_from_line(self,target,slope,y_int):
        return math.fabs((target.x * slope + target.y * -1 + y_int)) / math.sqrt(slope*slope + 1)

def ransac_line(data):
    global is_ransac_executed
    point_set = [Point()]
    point_set = data


    
    inliers = 0;
    best_match = Point_info(0,0) , Point_info(0,0)
    
    for i in range(0,3):
        index1 = int(random.uniform(0,(len(point_set)-1)))
    	index2 = int(random.uniform(0,(len(point_set)-1)))
    	point1 = point_set[index1]
    	point2 = point_set[index2]
    
   
   	first_point = Point_info(point1.x,point1.y)
    	second_point = Point_info(point2.x,point2.y)
    	slope,y_int = first_point.line_equation(second_point)

    	del point_set[index1]
    	del point_set[index2]
        current_inliers = 0
    	for point in point_set:
       		point_info = Point_info(point.x,point.y)
                dist = first_point.distance_from_line(point_info,slope,y_int)
       		if dist < 1.0:
                   current_inliers += 1
      
        	if current_inliers > inliers:
                	inliers = current_inliers
			best_match = first_point,second_point
                        rospy.loginfo("best match found : %d %d", first_point.x,first_point.y)

    rospy.loginfo("best match found : %d %d %d %d", best_match[0].x,best_match[0].y , best_match[1].x,best_match[1].y)
    showBestLine(best_match[0],best_match[1])
    #is_ransac_executed = True

def showBestLine(point1,point2):
    
    global rvizpub
    
    pointList = [Point()]
    pointmarker = Marker()
    pointmarker.header.frame_id = "/base_link"
    pointmarker.header.stamp = rospy.Time.now()
    pointmarker.type = Marker().LINE_STRIP
    pointmarker.action = Marker().ADD
    pointmarker.ns = "points"
    #pointmarker.id = x

    pointmarker.scale.x = 0.1
    pointmarker.scale.y = 0.1
    pointmarker.scale.z = 0.0
    pointmarker.color.a = 1.0
    pointmarker.color.r = 1.0
    pointmarker.color.g = 1.0
    pointmarker.color.b = 0.0
    
    point12 = Point()
    point12.x = point1.x
    point12.y = point1.y
    point12.z = 0.0
   
    point22 = Point()
    point22.x = point2.x
    point22.y = point2.y
    point22.z = 0.0
    
    
    
    pointmarker.points.append(point12)
    pointmarker.points.append(point22) 
    
    rvizpub.publish(pointmarker)
    
    
def laserDataReceived(data):
    global is_ransac_executed
    #rospy.loginfo("Callback called for new thing")
    #print "CALLING BACK"
    pointLists = [Point()]
    if is_ransac_executed:
 	is_ransac_executed = False
    	for x in range(0,(len(data.ranges)-1)):
        
        	rangeinfo = data.ranges[x]
        	th = (data.angle_min + float(x*data.angle_increment)) 
        
        	if rangeinfo != data.range_min and rangeinfo != data.range_max :
		
            		pointdata = Point()
            		pointdata.x = (rangeinfo* math.cos(th))
            		pointdata.y = (rangeinfo * math.sin(th))
            		pointLists.append(pointdata)
            	

    	ransac_line(pointLists)

def laserDataBroadcaster():
    rospy.init_node('laserDataBroadcaster', anonymous=True)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, laserDataReceived)
    #rospy.Subscriber("base_pose_ground_truth", Odometry, locationChanged)
    rospy.spin()
     

if __name__ == '__main__':
    try:
        laserDataBroadcaster()
    except rospy.ROSInterruptException:
        pass
