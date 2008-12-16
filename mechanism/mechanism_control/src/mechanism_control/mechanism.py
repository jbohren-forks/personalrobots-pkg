#! /usr/bin/env python
# Wrappers around the services provided by MechanismControlNode

import rostools
rostools.update_path('robot_srvs')
rostools.update_path('std_srvs')

import rospy, sys
from robot_srvs.srv import *
import std_srvs.srv

def list_controller_types():
    s = rospy.ServiceProxy('list_controller_types', ListControllerTypes)
    resp = s.call(ListControllerTypesRequest())
    for t in resp.types:
        print t

def list_controllers():
    rospy.wait_for_service('list_controllers')
    s = rospy.ServiceProxy('list_controllers', ListControllers)
    resp = s.call(ListControllersRequest())
    for c in resp.controllers:
        print c

def spawn_controller(xml):
    rospy.wait_for_service('spawn_controller')
    s = rospy.ServiceProxy('spawn_controller', SpawnController)
    resp = s.call(SpawnControllerRequest(xml))
    print ','.join([str(k) for k in resp.ok])
    for i in range(len(resp.ok)):
        if resp.ok[i] != 0:
            print "Spawned", resp.name[i]
        else:
            print "Error when spawning", resp.name[i]

def kill_controller(name):
    rospy.wait_for_service('kill_controller')
    s = rospy.ServiceProxy('kill_controller', KillController)
    resp = s.call(KillControllerRequest(name))
    if resp.ok == 1:
        print "Killed %s successfully" % name
    else:
        print "Error when killing", resp.ok


shutdown = rospy.ServiceProxy('shutdown', std_srvs.srv.Empty)

