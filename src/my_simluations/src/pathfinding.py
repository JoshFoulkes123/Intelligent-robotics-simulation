#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16, Int8
from my_simluations.srv import goal as _GOAL
from my_simluations.srv import goalResponse as _GOAL_resp
import time
import math


from my_simluations.srv import tables as _TABLES

completeAStarMap = []

def read_map():
    img = open('./src/my_simluations/data/cafe_map.pgm', 'rb')
    
    #Header values before width and height
    img.readline()
    img.readline()
    
    (width, height) = [int(i) for i in img.readline().split()]
    depth = int(img.readline())
    assert depth <= 255
    
    print(width)
    print(height)
    
    aStarMap = []
    
    for y in range(height):
        row = []
        for y in range(width):
            val = ord(img.readline(1))
            if val == 0:
                row.append(1)
            elif val == 254:
                row.append(0)   
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
    
def a_star_calculation(start, end):
    completeAStarMap = read_map()
#   print(completeAStarMap)
    width = len(completeAStarMap[0])
    height = len(completeAStarMap)
            
    openList = []
    closedList = []
    
    openList.append(aStarNode(None, (start['x'], start['y'])))
    
    while not len(openList) == 0:
    
        currentNode = openList[0]     
        currentNodeNum = 0
        
        for i in range(len(openList)-1):
            if openList[i].f < currentNode.f:
                currentNode = openList[i]
                currentNodeNum = i
                
        openList.pop(currentNodeNum)
        closedList.append(currentNode)        
        
        print('Start Node: (' + str(start['x']) + ',' + str(start['y']) + ')')
        print('End Node: (' + str(end['x']) + ',' + str(end['y']) + ')')
        print('Current Node: (' + str(currentNode.x) + ',' + str(currentNode.y) + ')')
        print('Size Open List: ' + str(len(openList)))
        print('Size Closed List: ' + str(len(closedList)))
        print()
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
                    
                if completeAStarMap[child_position[0]][child_position[1]] != -1:
                    continue
                    
                child = aStarNode(currentNode, child_position)
                child.parent = currentNode
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
                for node in closedList:
                    if node.x == children[i].x and node.y == children[i].y:
                        continue
                        
                children[i].g = currentNode.g + 1
                children[i].h = math.hypot(end['x'] - children[i].x, end['y']-children[i].y)
                children[i].f = children[i].g + children[i].h
                
                for open_node in openList:
                    if open_node.x == children[i].x and open_node.y == children[i].y:
                        if children[i].g > open_node.g:
                            continue
                        
                openList.append(children[i])
	            
def go_to_place(end):
    time.sleep(3)
    
    print(a_star_calculation({'x':10, 'y':50}, {'x':100, 'y': 200}))
    
    print("At destination:")
    print("x: " +str(end.x))
    print("y: " +str(end.y))
    print("yaw: " +str(end.yaw))
    return _GOAL_resp(100)

def pathfinder_server():
    completeAStarMap = read_map()
    rospy.init_node("pathfinder")
    rospy.loginfo("hello from pathfinder")
    s = rospy.Service("path", _GOAL, go_to_place)
    print("started")
    rospy.spin()

if __name__ == "__main__":
    pathfinder_server()

    
#def path_to_target():
#    rospy.wait_for_service("/queuemanager")
#    tables = rospy.ServiceProxy('/queuemanager', _TABLES)
#    print(tables(0))
#    print("pathing")
#    
#if __name__ == "__main__":
#    rospy.init_node("pathfinder")
#    rospy.loginfo("hello from pathfinder")
#    
#    aStarMap = read_map()
#    path_to_target()
#    #rospy.Service("/path", _TABLES, path_to_target)
    
#    rospy.spin()
