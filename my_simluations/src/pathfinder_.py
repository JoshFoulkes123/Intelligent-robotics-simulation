#!/usr/bin/env python3

from os import path
import os
from pickle import NONE
import numpy as np
#from my_simluations.src.transformations import pixel_to_world, world_to_pixel
import rospy
import rospkg
from std_msgs.msg import UInt16, Int8, String
from my_simluations.srv import goal as _GOAL
from my_simluations.srv import goalResponse as _GOAL_resp
from visualization_msgs.msg import MarkerArray, Marker
import time
import math
from transformations import world_to_pixel, pixel_to_world, worldtheta_to_pixeltheta, real_to_pixel, pixel_to_real, getHeading, rotateQuaternion, quaternion_to_euler_angle
import actionlib
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from geometry_msgs.msg import Twist , Point , Quaternion, Pose, PoseWithCovarianceStamped
from actionlib_msgs.msg import *
import tf

from my_simluations.srv import tables as _TABLES



completeAStarMap = []
threshhold = 0.4

ROBOT_SPEED = 0.4
ANGULAR_VELOCITY = 0.4


def read_map():
    img = open('/home/mandarpatil/robotics/src/my_simluations/data/cafe_map.pgm', 'rb')
    #img = open('/home/josh/catkin_ws/src/my_simluations/data/cafe_map.pgm', 'rb')

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


        #print('Current Node: ' + currentNode.toString(completeAStarMap))


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


                children.append(child)



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



def a_star_cleanup(aStarNodes):
    completeAStarMap = read_map()
    width = len(completeAStarMap[0])
    height = len(completeAStarMap)

    toKeep = [0]
    nextI = 0

    for i in range(len(aStarNodes)):
        complete = False
        if i == nextI:
            for j in (range(i+1, len(aStarNodes))):
                nodeOne = aStarNodes[i]
                nodeTwo = aStarNodes[j]

                trajectoryNormalise = (math.hypot(nodeOne['x'] - nodeTwo['x'], nodeOne['y']-nodeTwo['y']) / 10)
                trajectory = (int((nodeTwo['x'] - nodeOne['x']) / trajectoryNormalise),int((nodeTwo['y'] - nodeOne['y']) / trajectoryNormalise))

                if trajectory[0] != 0 and trajectory[1] != 0:
                    for x in range(nodeOne['x'] + 2, nodeTwo['x'], int(trajectory[0])):
                        for y in range(nodeOne['y'] + 2, nodeTwo['y'], int(trajectory[1])):
                            if completeAStarMap[int(y)][int(x)] != 1:
                                toKeep.append(j - 1)
                                nextI = j - 1
                                complete = True
                                break

                        if complete:
                            break
                elif trajectory[0] == 0:
                    for y in range(nodeOne['y'] + 2, nodeTwo['y'], int(trajectory[1])):
                        if completeAStarMap[int(y)][int(nodeOne['x'] )] != 1:
                            toKeep.append(j - 1)
                            nextI = j - 1
                            complete = True
                            break
                elif trajectory[1] == 0:
                    for x in range(nodeOne['x'] + 2, nodeTwo['x'], int(trajectory[0])):
                        if completeAStarMap[int(nodeOne['y'])][int(x)] != 1:
                            toKeep.append(j - 1)
                            nextI = j - 1
                            complete = True
                            break

                if complete:
                    break

    clean_a_star = []
    for i in toKeep:
        clean_a_star.append(aStarNodes[i])
    clean_a_star.append(aStarNodes[len(aStarNodes) -1])

    return clean_a_star


def angular_difference(pose_actual, pose_goal):

	return math.atan2(pose_goal.position.y-pose_actual.position.y, pose_goal.position.x-pose_actual.position.x) - getHeading(pose_actual.orientation)


def stop(cmd_vel):

	cmd_vel.linear.x = 0
	cmd_vel.angular.z = 0

#move_base code
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
   # print(getHeading(amcl_pose.orientation))
   # print(amcl_pose)

def rotate_bot(goal_heading):
    curr_heading = getHeading(amcl_pose.orientation)
    turn = Twist()
    print(curr_heading)
    print(goal_heading+goal_heading*0.1)
    print(goal_heading-goal_heading*0.1)
    while curr_heading > (goal_heading+goal_heading*threshhold) or curr_heading < (goal_heading-goal_heading*threshhold):
        msg = rospy.wait_for_message('/amcl_pose',PoseWithCovarianceStamped, timeout=None)
        curr_heading = getHeading(msg.pose.pose.orientation)
        print(curr_heading)
        print(goal_heading)
        ratio = abs(curr_heading-goal_heading)
        turn.angular.z = min(ANGULAR_VELOCITY, ANGULAR_VELOCITY*ratio)
        move.publish(turn)


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



    paths =a_star_calculation(start,end)
    paths = a_star_cleanup(paths)
    print(paths)
    previous_coordinate = Pose()
    previous_coordinate = amcl_pose
    twist = Twist()
    twist.linear.x = ROBOT_SPEED
    pose_goal = Pose()
    global move
    r = rospy.Rate(4)
    for i,coordinate in enumerate(paths):


        # pose_goal.position.x = coordinate['x']
        # pose_goal.position.y = coordinate['y']
        pose_goal.position.x,pose_goal.position.y  = pixel_to_real(coordinate['x'],coordinate['y'])
        #euler = tf.transformations.euler_from_quaternion(amcl_pose.orientation)
        roll,pitch,yaw = quaternion_to_euler_angle(previous_coordinate.orientation.x,previous_coordinate.orientation.y,previous_coordinate.orientation.z,previous_coordinate.orientation.w)
        print(yaw)
        #time.sleep(100)
        #heading = angular_difference(previous_coordinate, pose_goal)
        #print(pose_goal.position.x)
        #pixel to real

        inc_x = int(pose_goal.position.x) - int(previous_coordinate.position.x)
        inc_y = int(pose_goal.position.y) - int(previous_coordinate.position.y)

        # print(inc_x)
        # print(inc_y)
        angle_to_goal = math.atan2(inc_y, inc_x)
        print("angle to goal ",angle_to_goal)
        #rotate_bot(angle_to_goal)
        #pose_goal.orientation = rotateQuaternion(pose_goal.orientation,heading)
        #twist.angular.z = heading
        dist_to_goal = np.sqrt(inc_x*inc_x+inc_y*inc_y)
        print("dist_to_goal :",dist_to_goal)
        # previous_coordinate = pose_goal
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        if dist_to_goal >= 1 :
            
            if abs(angle_to_goal-yaw) >= 2:
                twist.linear.x = 0.0
                twist.angular.z = 0.15
                move.publish(twist)
                time.sleep(1)
            
            twist.linear.x = 0.08
            twist.angular.z = 0.0
            move.publish(twist)
            time.sleep(1)
            print("reached pose")
            previous_coordinate = amcl_pose

        r.sleep()
        





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

    move = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    time.sleep(1)
    print("wait for server to come up")
    rospy.init_node("pathfinder")
    rospy.loginfo("hello from pathfinder")
    #move_base.wait_for_server(rospy.Duration(5)) #takes up to five seconds
    print("Starting PathFinding and Navigation service")
    pathfinder_server()
