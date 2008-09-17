import rostools
rostools.update_path('mechanism_control')
rostools.update_path('generic_controllers')


import rospy, sys
from mechanism_control.srv import *
from generic_controllers.srv import *

def list_controllers():
    s = rospy.ServiceProxy('list_controllers', ListControllers)
    resp = s.call(ListControllersRequest())
    for t in resp.controllers:
        print t

def set_controller(controller, command):
    s = rospy.ServiceProxy(controller + '/set_command', SetCommand)
    resp = s.call(SetCommandRequest(command))
    print resp.command

def set_controller_vector(controller, command):
    s = rospy.ServiceProxy(controller + '/set_command', SetVectorCommand)
    resp = s(*command)

def get_controller(controller):
    s = rospy.ServiceProxy(controller + '/get_actual', GetActual)
    resp = s.call(GetActualRequest())
    print str(resp.time) + ": " + str(resp.command)

def set_position(controller, command):
    s = rospy.ServiceProxy(controller + '/set_position', SetPosition)
    resp = s.call(SetPositionRequest(command))
    print resp.command

def get_position(controller):
    s = rospy.ServiceProxy(controller + '/get_position', GetPosition)
    resp = s.call(GetPositionRequest())
    print str(resp.time) + ": " + str(resp.command)

def set_profile(controller, upper_turnaround, lower_turnaround, upper_decel_buffer, lower_decel_buffer):
    s = rospy.ServiceProxy(controller + '/set_profile', SetProfile)
    resp = s.call(SetProfileRequest(upper_turnaround, lower_turnaround, upper_decel_buffer, lower_decel_buffer))
    print str(resp.time)

