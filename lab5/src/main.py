#!/usr/bin/env python
import math
from heapq import *

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


orient = None
pos = None

# Map 
# Grid representation with 1s and 0s 
# 1 indicating an obstacle in that cell and 0 representing an empty cell.
map = np.array([
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]])


# Euclidian Distance
def get_euc_distance(point1,point2):
    euc_distance = ((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)**0.5
    return euc_distance


class main():

    def __init__(self):
        rospy.init_node('A_Path_Planning')
        rate = rospy.Rate(10)
        cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=100)
        pose = rospy.Subscriber("/base_pose_ground_truth", Odometry, callback=self.pose_callback)

        # Initial Start and Goal Position according to the world
        initial_pos_x,initial_pos_y = -8,-2
        goalx,goaly = self.get_parameters()

        # Get positions on grid
        final_startx,final_starty = self.new_coordinates_on_grid(initial_pos_x,initial_pos_y)
        final_goalx,final_goaly = self.new_coordinates_on_grid(goalx,goaly)

        # Get the Path using A* Path Finding Algorithm 
        final_start = final_startx,final_starty
        final_goal = final_goalx,final_goaly

        print("Start Position : {},{}".format(final_startx,final_starty))
        print("Goal Position : {},{}".format(final_goalx,final_goaly))

        path_planned = astar(map,final_start,final_goal)
        currentx,currenty = path_planned[0][0],path_planned[0][1]
        print("Starting Point : {},{}".format(path_planned[0][0],path_planned[0][1]))

        for i in range(0,12):
            rotate_ = Twist()
            rotate_.angular.z = -(math.pi/4.0)
            cmd_vel.publish(rotate_)
            rate.sleep()
        
        for i in path_planned:
            print("Current : {},{}".format(currentx,currenty))
            print("Next Node : {},{}".format(i[0],i[1]))
            next_node = i[0],i[1]
            current = currentx,currenty
            roll,pitch,yaw = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            distance = get_euc_distance(current,next_node)
            angle_of_rotation = math.atan2(next_node[1]-currenty,next_node[0]-currentx) - yaw
            self.move(cmd_vel,distance,angle_of_rotation)
            currentx = pos.x
            currenty = pos.y

    def move(self,cmd_vel,distance,angle_of_rotation):
        vel = Twist()
        r = rospy.Rate(5)
        for i in range(10):
            vel.linear.x = distance/2
            vel.angular.z = angle_of_rotation/2.4
            cmd_vel.publish(vel)
            r.sleep()


    def new_coordinates_on_grid(self,x,y):
        x_new = x + 9
        y_new = 9 - y
        return int(x_new),int(y_new)

    def pose_callback(self,path):
        global pos
        global orient
        pos = path.pose.pose.position
        orient = path.pose.pose.orientation

    # Get Parameters from the launch file.
    def get_parameters(self):
        goalx_param = rospy.search_param('goalx')
        goaly_param = rospy.search_param('goaly')
        goalx = rospy.get_param(goalx_param)
        goaly = rospy.get_param(goaly_param)
        print('Goal entered: {},{}'.format(str(goalx),str(goaly)))

        # Conversion of Float to int
        goalx_int = int(goalx)
        goaly_int = int(goaly)
        return (goalx_int,goaly_int)

def astar(array, start, goal):

    closed,previous,open_s = set(),{},[]
    g,f = {start:0},{start:get_euc_distance(start, goal)}
    heappush(open_s, (f[start], start))

    while open_s:

        current = heappop(open_s)[1]

        if current == goal:
            path = []
            new_path = []
            while current in previous:
                path.append(current)
                current = previous[current]
            path.reverse()
            for point in path:
                y = math.floor(9 - point[1])
                x = math.floor(point[0] - 8)
                new_path.append((x,y))
            return new_path

        closed.add(current)
        Alternativ_condition(current, g, array, closed, open_s, previous, goal, f)

    return False


def Alternativ_condition(current, g, array, closed, open_s, previous, goal, f):
    x,y = current
    list = []
    A = [0,1,-1]
    B = [0,1,-1]

    for i in A:
        for j in B:
            new = (i,j)
            if (new[0],new[1]) != (0,0):
                list.append(new)

    for i, j in list:
        neighbours = x + int(i), y + int(j)
        new_g_score = g[current] + get_euc_distance(current, neighbours)
        condition1 = 0 <= neighbours[0] < array.shape[1]
        condition2 = 0 <= neighbours[1] < array.shape[0]
        condition3 = array[neighbours[1]][neighbours[0]]

        if (condition1 and condition2 and condition3) or (not condition1) or (condition1 and not condition2):
            continue

        if neighbours in closed:
            if new_g_score >= g.get(neighbours, 0):
                continue

        next_condition = [i[1]for i in open_s]
        if  new_g_score < g.get(neighbours, 0) or neighbours not in next_condition:
            previous[neighbours] = current
            g[neighbours] = new_g_score
            f[neighbours] = new_g_score + get_euc_distance(neighbours, goal)
            heappush(open_s, (f[neighbours], neighbours))

    

if __name__ == '__main__':
    if not rospy.is_shutdown():
        try:
            main()
        except rospy.ROSInterruptException as e:
            print('Exception Occured :: {}'.format(str(e)))
