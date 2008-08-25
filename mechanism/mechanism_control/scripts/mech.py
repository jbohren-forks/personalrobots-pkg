#! /usr/bin/env python
# Provides quick access to the services exposed by MechanismControlNode

import rostools
rostools.update_path('mechanism_control')

import rospy, sys
from mechanism_control.srv import *

def print_usage(exit_code = 0):
    print '''Commands:
    lt  - List controller Types
    lc  - List active controllers
    sp  - Spawn a controller using the xml passed over stdin
    kl <name>  - Kills the controller named <name>'''
    sys.exit(exit_code)

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

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage()

    if sys.argv[1] == 'lt':
        list_controller_types()
    elif sys.argv[1] == 'lc':
        list_controllers()
    elif sys.argv[1] == 'sp':
        xml = ""
        if len(sys.argv) > 2:
            f = open(sys.argv[2])
            xml = f.read()
            f.close()
        else:
            xml = sys.stdin.read()
        spawn_controller(xml)
    elif sys.argv[1] == 'kl':
        kill_controller(sys.argv[2])
