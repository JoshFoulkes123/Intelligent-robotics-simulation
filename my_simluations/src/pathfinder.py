#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16, Int8
from my_simluations.srv import goal as _GOAL
from my_simluations.srv import goalResponse as _GOAL_resp
import time

def go_to_place(req):
    time.sleep(3)
    print("At destination:")
    print("x: " +str(req.x))
    print("y: " +str(req.y))
    print("yaw: " +str(req.yaw))
    return _GOAL_resp(100)

def pathfinder_server():
    rospy.init_node("pathfinder")
    rospy.loginfo("hello from pathfinder")
    s = rospy.Service("path", _GOAL, go_to_place)
    print("started")
    rospy.spin()

if __name__ == "__main__":
    pathfinder_server()
