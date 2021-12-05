#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String 
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from geometry_msgs.msg import Twist , Point
from actionlib_msgs.msg import *



locations={"cafe_table":[2, -8, 0.2, 0, -0, 0],
            "cafe_table_0":[2.0793, -3.32335, 0.2, 0, -0, 0],
            "cafe_table_1":[-1.05513, -3.35734, 0.2, 0, -0, 0],
            "cafe_table_2":[-1.12287, -8.03356, 0.2, 0, -0, 0],
            "kitchen":[-3.23722,6.393821, 0.373934,0, -0,0]
            }
           

def GOTO(place):
    coordinates=locations[place]
    x=coordinates[0]
    y=coordinates[1]
    ox=coordinates[2]
    oy=coordinates[3]
    oz=coordinates[4]
    ow=coordinates[5]
    move(x,y,ox,oy,oz,ow)

def move(x,y,ox,oy,oz,ow):
    ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)

    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("waiting for move base server")
    goal=MoveBaseGoal()

    #frame parameters
    goal.target_pose.header.frame_id="map"
    goal.target_pose.header.stamp = rospy.Time.now()

    #goal

    goal.target_pose.pose.position= Point(x,y,0)
    goal.target_pose.pose.orientation.x=ox
    goal.target_pose.pose.orientation.y=oy
    goal.target_pose.pose.orientation.z=oz
    goal.target_pose.pose.orientation.w=ow

    ac.send_goal(goal)

if __name__ == '__main__':
    rospy.init_node("navigation_node",anonymous=False)
    
    GOTO("kitchen")
    
    
    #!/usr/bin/env python3
