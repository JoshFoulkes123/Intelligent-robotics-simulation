#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16, Int8
from my_simluations.srv import tables as _TABLES
from my_simluations.srv import tablesResponse as _TABLES_resp

def return_table(req):

    _deliver = 3
    _clean = 5
    print("tables_sent")
    return _TABLES_resp(_deliver,_clean)

def return_tables_server():
    rospy.init_node("table_server")
    rospy.loginfo("hello from server")
    s = rospy.Service("tables", _TABLES, return_table)
    print("returning tables")
    rospy.spin()

if __name__ == "__main__":
    return_tables_server()
