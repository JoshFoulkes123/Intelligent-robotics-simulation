#!/usr/bin/env python3

from os import path
import os
from pickle import NONE
#from my_simluations.src.transformations import pixel_to_world, world_to_pixel
import rospy
import rospkg
from std_msgs.msg import UInt16, Int8, String
from my_simluations.srv import goal as _GOAL
from my_simluations.srv import goalResponse as _GOAL_resp
from visualization_msgs.msg import MarkerArray, Marker
import time
import math
from transformations import world_to_pixel, pixel_to_world, worldtheta_to_pixeltheta, real_to_pixel, pixel_to_real
import actionlib
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from geometry_msgs.msg import Twist , Point , Quaternion, Pose, PoseWithCovarianceStamped
from actionlib_msgs.msg import *

from my_simluations.srv import tables as _TABLES


#start ={"x":0,"y":0}
completeAStarMap = []


def read_map():
    img = open('/home/josh/catkin_ws/src/my_simluations/data/cafe_map.pgm', 'rb')

    #Header values before width and height
    img.readline()
    img.readline()

    (width, height) = [int(i) for i in img.readline().split()]
    depth = int(img.readline())
    assert depth <= 255

    aStarMap = []

    for y in range(height):
        row = []
        for y in range(width):
            val = ord(img.readline(1))
            if val < 100:
                row.append(0)
            elif val == 254:
                row.append(1)
            else:
                row.append(-1)
        aStarMap.append(row)

    #print(aStarMap)
    print("map made")
    return aStarMap



class aStarNode():
    def __init__(self, parentNode = None, position = None):
        self.parentNode = parentNode
        self.position = position
        self.x = position[0]
        self.y = position[1]

        self.h = 0
        self.g = 0
        self.f = 0

    def toString(self, completeAStarMap = None):
        if completeAStarMap == None:
            return '(' + str(self.x) + ',' + str(self.y) + ') [' + str(self.g) + ',' + str(self.h) + ',' + str(self.f)+  ']'
        else:
            return '(' + str(self.x) + ',' + str(self.y) + ') [' + str(self.g) + ',' + str(self.h) + ',' + str(self.f) + '] {' + str(completeAStarMap[self.y][self.x]) + '}'


def a_star_calculation(start, end):
    completeAStarMap = read_map()
#   print(completeAStarMap)
    width = len(completeAStarMap[0])
    height = len(completeAStarMap)

    print(width)
    print(height)

    openList = []
    closedList = []

    openList.append(aStarNode(None, (start['x'], start['y'])))

    while not (len(openList) == 0 or len(openList) > 10000000):

        currentNode = openList[0]
        currentNodeNum = 0

        for i in range(len(openList)):
            if openList[i].f < currentNode.f:
                currentNode = openList[i]
                currentNodeNum = i

        openList.pop(currentNodeNum)
        closedList.append(currentNode)

  #      print('Start Node: (' + str(start['x']) + ',' + str(start['y']) + ')')
 ##       print('End Node: (' + str(end['x']) + ',' + str(end['y']) + ')')
##       print('Current Node: (' + str(currentNode.x) + ',' + str(currentNode.y) + ')')
        #print('Current Node: ' + currentNode.toString(completeAStarMap))
        #print('Size Open List: ' + str(len(openList)))
#        print('Size Closed List: ' + str(len(closedList)))
#        print()
#        print("OPEN LIST")
#        for node in openList:
#            print('(' + str(node['x']) + ',' + str(node['y']) + ')')
#        print()
#        print("CLOSED LIST")
#        for node in closedList:
#            print('(' + str(node['x']) + ',' + str(node['y']) + ')')
#        print()
#        time.sleep(1)

        if currentNode.x == end['x'] and currentNode.y == end['y']:
            path_to_end = []
            current_node = currentNode
            while current_node != None:
                path_to_end.append({'x':current_node.x, 'y':current_node.y})
                current_node = current_node.parentNode
            return path_to_end[::-1]
        else:
            children = []

            for child_position_rel in [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]:
                child_position = (currentNode.x + child_position_rel[0], currentNode.y + child_position_rel[1])

                if child_position[0] < 0 or child_position[0] >= width or child_position[1] < 0 or child_position[1] >= height:
                    continue

                if completeAStarMap[child_position[1]][child_position[0]] != 1:
                    continue

                child = aStarNode(currentNode, child_position)
                child.parent = currentNode

#                print(child.toString())
                children.append(child)

#            if currentNode['x'] > 0:
#                if currentNode['y'] > 0:
#                    children.append(a_star_nodes[x-1][y-1])
#                if currentNode['y'] < height - 1:
#                    children.append(a_star_nodes[x-1][y+1])
#            if currentNode['x'] < width - 1:
#                children.append(a_star_nodes[x+1][y])
#                if currentNode['y'] > 0:
#                    children.append(a_star_nodes[x+1][y-1])
#                if currentNode['y'] < height - 1:
#                    children.append(a_star_nodes[x+1][y+1])
#            if currentNode['y'] > 0:
#                children.append(a_star_nodes[x][y-1])
#            if currentNode['y'] < height - 1:
#                children.append(a_star_nodes[x][y+1])

            for i in range(len(children)):
                skip = False
                for node in closedList:
                    if node.x == children[i].x and node.y == children[i].y:
                        skip = True
                        break

                if skip:
                    continue

                children[i].g = currentNode.g + math.hypot(children[i].x - currentNode.x, children[i].y - currentNode.y)
                children[i].h = math.hypot(end['x'] - children[i].x, end['y']-children[i].y)
                children[i].f = children[i].g + children[i].h

                for open_node in openList:
                    if open_node.x == children[i].x and open_node.y == children[i].y:
                        if children[i].g >= open_node.g:
                            skip = True
                            break

                if skip:
                    continue

                openList.append(children[i])




# def slope(p1, p2):
# 	delta_y = p2[1]-p1[1]
# 	delta_x = p2[0]-p1[0]
# 	return delta_y/delta_x if delta_x!=0 else float('inf')
#
# def euclidean_distance():
#
# 	return math.sqrt(math.pow((self.goal[0]-self.pos.position.x),2) + math.pow((self.goal[1]-self.pos.position.y),2))
#
# def angular_difference():
#
# 	return math.atan2(self.goal[1]-self.pos.position.y, self.goal[0]-self.pos.position.x) - self.theta
#
#
# def stop():
#
# 	self.vel_msg.linear.x = 0
# 	self.vel_msg.angular.z = 0
# 		self.vel_publisher.publish(self.vel_msg)


def navigate(x,y,yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(x,y,0.0),Quaternion(0,0,0,1)) ##sending according to amcl
    global move_base
    move_base.send_goal(goal)

    #Allow the waiter to complete the task..need to vary the seconds accordingly
    success = move_base.wait_for_result(rospy.Duration(5))
    state = move_base.get_state()

    res = False
    if success and state == GoalStatus.SUCCEEDED:
        res = True
    else:
        move_base.cancel_goal()
    return res

def amcl_callback(msg):
    print("inside amcl callback")
    global amcl_pose
    amcl_pose = msg.pose.pose
    print(amcl_pose)

def go_to_place(request):
    time.sleep(3)
    x,y= real_to_pixel(request.x,request.y)
    yaw= request.yaw

    end = {'x':x,'y':y}
    start = None
    #get the start pose from amcl
    global amcl_pose
    amcl_pose = None

    sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,amcl_callback)
    r = rospy.Rate(1)
    r.sleep()

    print(amcl_pose)
    if amcl_pose is not None:
        print("inside amcl_pose not NONE")
        _x_,_y_ = real_to_pixel(amcl_pose.position.x,amcl_pose.position.y)
        start= {'x': _x_,'y': _y_}
        _start = {'x':amcl_pose.position.x,'y': amcl_pose.position.y}
    print(_start)
    print(start)
    print(end)

    # global completeAStarMap
    # global width, height
    # completeAStarMap = read_map()
    # width = len(completeAStarMap[0])
    # height = len(completeAStarMap)
    #start is in terms of world coordinates ..
    # image_size = (height,width)
    # world_points=(amcl_pose.position.x,amcl_pose.position.y)
    # pixel_points=world_to_pixel(world_points,image_size)
    # start = {'x':pixel_points[0],'y':pixel_points[1]}

    paths =a_star_calculation(start,end)
    #paths =a_star_calculation({'x':101, 'y':230}, {'x':190, 'y':275})
    #print(paths)
    #time.sleep(1000)
    #waht about the yaw
    previous_coordinate = start
    for i,coordinate in enumerate(paths):
        #rospy.loginfo("Navigating to pose",coordinate)
        #the coordinates are in pixel
        # world_points =pixel_to_world((coordinate['x'],coordinate['y']),image_size)
        # success = navigate(world_points[0],world_points[1],amcl_pose.orientation)
        success = navigate(coordinate['x'],coordinate['y'],0)
        previous_coordinate = coordinate
        if not success:
            print("failed to reach pose",coordinate)
            time.sleep(1)
            go_to_place(request)
            break
            # wait for a few second
            #get the current coordinates and call go_to_place(req)
        print("reached pose")



    print("At destination:")
    print("x: " +str(end.x))
    print("y: " +str(end.y))
    print("yaw: " +str(end.yaw))
    return _GOAL_resp(100)

def pathfinder_server():
    #completeAStarMap = read_map()

    #print(a_star_calculation({'x':10, 'y':50}, {'x':100, 'y': 200}))
    s = rospy.Service("path", _GOAL, go_to_place)
    print("started")
    rospy.spin()

if __name__ == "__main__":
    amcl_pose = None
    # completeAStarMap = []
    # image_size = None
    # width =  0
    # height =0

    move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    print("wait for server to come up")
    rospy.init_node("pathfinder")
    rospy.loginfo("hello from pathfinder")
    move_base.wait_for_server(rospy.Duration(5)) #takes up to five seconds
    print("Starting PathFinding and Navigation service")
    pathfinder_server()
