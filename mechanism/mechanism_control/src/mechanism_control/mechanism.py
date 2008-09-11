#! /usr/bin/env python
# Wrappers around the services provided by MechanismControlNode

import rostools
rostools.update_path('mechanism_control')
rostools.update_path('std_srvs')

import rospy, sys
from mechanism_control.srv import *
import std_srvs.srv

def list_controller_types():
    s = rospy.ServiceProxy('list_controller_types', ListControllerTypes)
    resp = s.call(ListControllerTypesRequest())
    for t in resp.types:
        print t

def list_controllers():
    s = rospy.ServiceProxy('list_controllers', ListControllers)
    resp = s.call(ListControllersRequest())
    for c in resp.controllers:
        print c

def spawn_controller(xml):
    rospy.wait_for_service('spawn_controller')
    s = rospy.ServiceProxy('spawn_controller', SpawnController)
    resp = s.call(SpawnControllerRequest(xml))
    if resp.ok == 1:
        print "Spawned successfully"
    else:
        print "Error when spawning", resp.ok

def kill_controller(name):
    s = rospy.ServiceProxy('kill_controller', KillController)
    resp = s.call(KillControllerRequest(name))
    if resp.ok == 1:
        print "Killed %s successfully" % name
    else:
        print "Error when killing", resp.ok


shutdown = rospy.ServiceProxy('shutdown', std_srvs.srv.Empty)

