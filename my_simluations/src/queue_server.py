#!/usr/bin/env python

from __future__ import print_function
from genpy import message
import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


def trigger_response(request):
    '''
    
    it returns a trigger response

    '''

    return TriggerResponse(success= True, message= "Go to table")



rospy.init_node('queue_server')

serv_obj = rospy.Service('/queue_server',Trigger,trigger_response)

rospy.spin()