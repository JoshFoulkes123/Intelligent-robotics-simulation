#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16, Int8
from my_simluations.srv import goal as _GOAL
from my_simluations.srv import tables as _TABLES
import time

#drop off points for tables; format x,y,yaw
tables_drop_off = [[1,2,1.2],
          [2.1,3,1.4]]

#print(tables_drop_off[1][2])
#reminder of how it works
# goal = _GOAL()
# goal.x = 2

#tables_receive = rospy.Service('/tables_', _TABLES, )

def request_tables_client():
    rospy.wait_for_service('tables')
    flipper = 1
    try:

        tables_return = rospy.ServiceProxy('tables', _TABLES)
        resp = tables_return(flipper)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def request_location_client(_x, _y, _yaw):
    rospy.wait_for_service('path')

    try:
        goal_return = rospy.ServiceProxy('path', _GOAL)
        resp = goal_return(_x,_y,_yaw)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node("main")
    rospy.loginfo("hello from main")
    y = request_tables_client()
    print("Tables to deliver: "+ str(y.table_deliver))
    print("Tables to clean: "+ str(y.table_clean))

    x = tables_drop_off[0][0]
    y = tables_drop_off[0][1]
    yaw = tables_drop_off[0][2]
    time.sleep(2)
    z = request_location_client(x,y,yaw)
    print("goal response: " + str(z.reached))

    x = tables_drop_off[1][0]
    y = tables_drop_off[1][1]
    yaw = tables_drop_off[1][2]
    time.sleep(2)
    z = request_location_client(x,y,yaw)
    print("goal response: " + str(z.reached))

    rospy.spin()
