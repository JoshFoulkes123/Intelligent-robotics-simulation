#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16, Int8
from my_simluations.srv import goal as _GOAL
from my_simluations.srv import tables as _TABLES
import time

#drop off points for tables; format x,y,yaw
tables_drop_off = [[1,2,1.2],
          [2.1,3,1.4],[6,3,1.4],[2.1,3,1.4],[2.1,3,1.4],[2.1,3,1.4],[2.1,3,1.4],[2.1,3,1.4],]

kitchen_coordinates = [-1,-1.2,0]

#print(tables_drop_off[1][2])
#reminder of how it works
# goal = _GOAL()
# goal.x = 2

#tables_receive = rospy.Service('/tables_', _TABLES, )

def request_tables_client():
    rospy.wait_for_service('/queuemanager')
    flipper = 1
    try:

        tables_return = rospy.ServiceProxy('/queuemanager', _TABLES)
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

# def food_placed():
#     print("--Please place food to be delivered")# this will be displayed to the waitors via LCD or voice
#     #this part of the code blocks the code waiting for food to be added
#     #currently it just waits a short perioed of time before moving
#     #this is where the weight sensor would be used to detect an increase in the weight
#     #or a button onboard of the robot
#     time.sleep(4)
#     print("food placed")
#
# def food_taken():
#     print("--Please take your food and enjoy")# this will be displayed to the customers via LCD or voice
#     #this part of the code blocks the code waiting for food to be removed
#     #currently it just waits a short period of time before moving
#     #this is where the weight sensor would be used to dtect a decrease in the weight
#     #or a button onboard of the robot
#     ui_client("hello")
#     #time.sleep(4)
#     print("food removed")
#
#
# def clear_table():
#     print("--Please add empty plates")
#     #thius section is the part to clean the table
#     #this could be achived using a weight sensor for peole to add plates
#     #a rebotic arm could also be used
#     #or a button onboard of the robot
#     time.sleep(4)
#     print("table cleaned")




if __name__ == "__main__":
    rospy.init_node("main")
    rospy.loginfo("hello from main")
    print("starting loop")
    while not rospy.is_shutdown():
        tableToDeliverTo = -1
        tableToClean = -1
        #will loop infinitly until something needs to be done
        while tableToDeliverTo == -1 and tableToClean ==-1:
            print("waiting")
            tableInformation = request_tables_client()
            tableToDeliverTo = tableInformation.table_deliver
            tableToClean = tableInformation.table_clean
            time.sleep(1)
            print("tables_received")

        if(tableToDeliverTo != -1):
            print("going")
            request_location_client(tables_drop_off[tableToDeliverTo-1][0],tables_drop_off[tableToDeliverTo-1][1],tables_drop_off[tableToDeliverTo-1][2])
            time.sleep(5)
            print("arrived")
            time.sleep(1)


        if(tableToClean != -1):
            print("cleaning")
            print("going")
            request_location_client(tables_drop_off[tableToClean-1][0],tables_drop_off[tableToClean-1][1],tables_drop_off[tableToClean-1][2])
            time.sleep(5)
            print("arrived")
            time.sleep(1)

        if(tableToClean != -1 or tableToDeliverTo != -1):
            print ("returning home")
            request_location_client(kitchen_coordinates[0],kitchen_coordinates[1],kitchen_coordinates[2])
            time.sleep(5)
            print("at home")
            time.sleep(1)

        if(tableToClean != -1):
            print("no action taken")
            time.sleep(3)

    #
    # y = request_tables_client()
    # print("Tables to deliver: "+ str(y.table_deliver))
    # print("Tables to clean: "+ str(y.table_clean))
    #
    # x = tables_drop_off[0][0]
    # y = tables_drop_off[0][1]
    # yaw = tables_drop_off[0][2]
    # time.sleep(2)
    # z = request_location_client(x,y,yaw)
    # print("goal response: " + str(z.reached))
    #
    # x = tables_drop_off[1][0]
    # y = tables_drop_off[1][1]
    # yaw = tables_drop_off[1][2]
    # time.sleep(2)
    # z = request_location_client(x,y,yaw)
    # print("goal response: " + str(z.reached))

    #rospy.spin()
