#!/usr/bin/env python

import rostools
rostools.update_path('robot_mechanism_controllers')

import rospy, sys
from robot_mechanism_controllers.srv import *
from robot_srvs.srv import *
from std_msgs.msg import *
from time import sleep

class SendMessageOnSubscribeAndExit(rospy.SubscribeListener):
    def __init__(self, msg):
        self.msg = msg
        print "Waiting for subscriber..."

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peer_publish(self.msg)
        sleep(0.1)  # TODO: change this line when flushing messages is implemented
        rospy.signal_shutdown("Done")
        sys.exit(0)

def list_controllers():
    rospy.wait_for_service('list_controllers')
    s = rospy.ServiceProxy('list_controllers', ListControllers)
    resp = s.call(ListControllersRequest())
    for t in resp.controllers:
        print t

def set_controller(controller, command):
    rospy.init_node('control', anonymous = True)
    pub = rospy.Publisher('/' + controller + '/set_command', Float64,
                          SendMessageOnSubscribeAndExit(Float64(command)))
    rospy.spin()

def set_controller_vector(controller, command):
    rospy.init_node('control', anonymous = True)
    pub = rospy.Publisher('/' + controller + '/set_command', Vector3,
                          SendMessageOnSubscribeAndExit(Vector3(*command)))
    rospy.spin()

def get_controller(controller):
    rospy.wait_for_service(controller + '/get_command')
    s = rospy.ServiceProxy(controller + '/get_command', JointCmd)
    resp = s.call(JointCmdRequest())
    print str(resp.name) + ": " + str(resp.positions)+ ": " + str(resp.velocity)

def get_controller_vector(controller):
    rospy.wait_for_service(controller + '/get_actual')
    s = rospy.ServiceProxy(controller + '/get_actual', GetVector)
    resp = s()
    print "(%f, %f, %f)" % (resp.v.x, resp.v.y, resp.v.z)

def set_position(controller, command):
    rospy.wait_for_service(controller + '/set_position')
    s = rospy.ServiceProxy(controller + '/set_position', SetPosition)
    resp = s.call(SetPositionRequest(command))
    print resp.command

def get_position(controller):
    rospy.wait_for_service(controller + '/get_position')
    s = rospy.ServiceProxy(controller + '/get_position', GetPosition)
    resp = s.call(GetPositionRequest())
    print str(resp.time) + ": " + str(resp.command)


