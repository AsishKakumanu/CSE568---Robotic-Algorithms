#!/usr/bin/env python
import math

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sympy import geometry as sp
from visualization_msgs.msg import Marker

# Declarations

points = []
position = 0
orientation = 0
ranges = np.zeros((361, 2))
obstacle = False
adjacent_line = False
on_line = False


def callback(data):
    global ranges
    global obstacle, adjacent_line
    data_range = data.ranges
    data_range = np.array(data_range)
    flagCount = 0
    max_angle = 150

    # Scan the front of the robot .i.e., 120 - 240 points.
    # Obstacle Scanning in front of the Robot.

    for i in range(max_angle):
        if data_range[max_angle+i] < 1:
            flagCount += 1

    if flagCount > 1:
        obstacle = 1
    else:
        obstacle = 0

    # Scan the sides of the robot.
    # Line Scanning to the sides of the robot.
    # As the Robot moves mostly to the left. Let's Scan the left side of the robot.

    flagCount = 0
    minangle = 80

    for i in np.arange(max_angle):
        if data_range[i] < 1:
            flagCount += 1

    if flagCount > 10:
        adjacent_line = 1
    else:
        adjacent_line = 0


def callback_odom(data):
    global orientation
    global position
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation


def angular_velocity(goal_angle):
    global obstacle, on_line
    if on_line:
        return min(goal_angle, 1)
    elif on_line:
        return 1
    else:
        return min(goal_angle, 1)


def velocity_line_follow(goal_angle):
    global adjacent_line, obstacle
    if obstacle:
        return 0.5
    if adjacent_line:
        return 0
    else:
        return -1 * 0.3


def check_point_coordiantes(points):
    global position

    # Numpy Array
    global pos
    A = points[0, :]
    B = points[1, :]
    C = np.array([position.x, position.y])
    area = abs((A[0] * (B[1] - C[1]) + B[0] *
                (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0)
    threshold = 0.6
    rospy.loginfo(area)
    if (area < threshold):
        return 1
    else:
        return 0


def bug2():

    # Init 
    global obstacle, orientation, position, on_line
    ## Publisher & Subscriber
    cmd_vel_publisher, freq_rate = pubsub()
    points = np.array([[-8, -2], [4.5, 9.0]])
    Goal = False
    thresholdDistance = 0.5
    State = "GOAL_SEEK"

    while not Goal:
        if orientation != 0:
            angle = radangle()
            current_distance = math.sqrt(
                (points[1, 0] - position.x) ** 2 + (points[1, 1] - position.y) ** 2)
            goal_angle = angular_vel(points, angle)
            on_line = check_point_coordiantes(points)
            twist = Twist()
            if current_distance < thresholdDistance:
                twist.linear.x = 0
                twist.angular.z = 0
                break
            else:
                velocity = 0.0
                if obstacle:
                    velocity = 0.0
                else:
                    velocity = 0.6
                twist.linear.x = velocity
                if State == "GOAL_SEEK":
                    twist.angular.z = angular_velocity(goal_angle)
                    if obstacle:
                        State = "LINE_FOLLOW"

                else:
                    twist.angular.z = -1 * velocity_line_follow(goal_angle)
                    if on_line and not obstacle:
                        State = "GOAL_SEEK"
            cmd_vel_publisher.publish(twist)
            freq_rate.sleep()


def pubsub():
    rate = 10
    cmd_vel_publisher = rospy.Publisher(
        "/robot_0/cmd_vel", Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber(
        "/robot_0/base_pose_ground_truth", Odometry, callback_odom)
    freq_rate = rospy.Rate(rate)
    return cmd_vel_publisher, freq_rate


def angular_vel(points, angle):
    goal_angle = math.atan(
        (points[1, 1] - position.y)/(points[1, 0] - position.x)) - angle
    rospy.loginfo("goal_angle {}".format(goal_angle))
    print("angle {}".format(goal_angle))
    return goal_angle


def radangle():
    angle = 2 * np.arcsin(orientation.z)
    rospy.loginfo("angle {}".format(angle))
    print("angle {}".format(angle))
    return angle


if __name__ == '__main__':
    try:
        rospy.init_node('bug2', anonymous=True)
        rospy.Subscriber("/robot_0/base_scan", LaserScan, callback)
        bug2()
    except rospy.ROSInterruptException:
        pass
