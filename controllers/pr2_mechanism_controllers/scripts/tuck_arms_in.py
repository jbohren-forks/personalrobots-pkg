#!/usr/bin/python                                                               
# Software License Agreement (BSD License)                                      
#                                                                               
# Copyright (c) 2008, Willow Garage, Inc.                                       
# All rights reserved.                                                          
#                                                                               
# Redistribution and use in source and binary forms, with or without            
# modification, are permitted provided that the following conditions            
# are met:                                                                      
#                                                                               
#  * Redistributions of source code must retain the above copyright             
#    notice, this list of conditions and the following disclaimer.              
#  * Redistributions in binary form must reproduce the above                    
#    copyright notice, this list of conditions and the following                
#    disclaimer in the documentation and/or other materials provided            
#    with the distribution.                                                     
#  * Neither the name of the Willow Garage nor the names of its                 
#    contributors may be used to endorse or promote products derived            
#    from this software without specific prior written permission.              
#                                                                               
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS           
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT             
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS             
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE                
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,           
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,          
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;              
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER              
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT            
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN             
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE               
# POSSIBILITY OF SUCH DAMAGE.                                                   
#                                                                               

# Author: Kevin Watts                                                           

# Tuck arms in on PR2 for moving

import roslib
import copy
import threading
import sys, os
from time import sleep

# Loads interface with the robot.                                               
roslib.load_manifest('mechanism_bringup')
roslib.load_manifest('mechanism_control')
import rospy
from std_msgs.msg import *
from robot_srvs.srv import *

from robot_mechanism_controllers.srv import *
from robot_mechanism_controllers import controllers
from mechanism_control import mechanism

class SendMessageOnSubscribe(rospy.SubscribeListener):
    def __init__(self, msg):
        self.msg = msg
        print "Waiting for subscriber..."

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peer_publish(self.msg)
        sleep(0.1)

spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
#kill_controller = rospy.ServiceProxy('kill_controller', KillController)

def xml_for_hold(name, p, i, d, iClamp):
    return """                                                                 
<controller name="%s_controller" type="JointPositionControllerNode">       
<joint name="%s_joint">                                                       
<pid p="%d" i="%d" d="%d" iClamp="%d" />                                       
</controller>""" % (name, name, p, i, d, iClamp)

def hold_joint(name, p, i, d, iClamp, holding):
    # Try to launch 3x                                                          
    # If launched, add to list of holding controllers and return true           
    for i in range(1,4):
        try:
            resp = spawn_controller(xml_for_hold(name, p, i, d, iClamp))
            if ord(resp.ok[0]) != 0:
                holding.append(resp.name[0])
                return True
        except Exception, e:
            print "Failed to spawn holding controller %s on try %d" % (name, i)
    return False

def set_controller(controller, command):
    pub = rospy.Publisher('/' + controller + '/set_command', Float64,
                              SendMessageOnSubscribe(Float64(command)))

def hold_side(side, pan_angle, holding):
    hold_joint("%s_elbow_flex" % side, 50, 15, 8, 2, holding)
    set_controller("%s_elbow_flex_controller" % side, float(3.0))

    hold_joint("%s_upper_arm_roll" % side, 20, 2, 1.0, 1.0, holding)
    set_controller("%s_upper_arm_roll_controller" % side, float(0.0))

    hold_joint("%s_shoulder_lift" % side, 35, 7, 4, 3, holding)
    set_controller("%s_shoulder_lift_controller" % side, float(3.0))

    hold_joint("%s_shoulder_pan" % side, 70, 6, 8, 4, holding)
    set_controller("%s_shoulder_pan_controller" % side, float(pan_angle))

def main():
    usage = "Usage: tuck_arms_in.py <arms> ; <arms> is \'(r)ight\', \'(l)eft\', or \'(b)oth arms\'"

    if len(sys.argv) < 2:
        print usage
        sys.exit(1)
    arms = sys.argv[1]

    rospy.wait_for_service('spawn_controller')
    rospy.init_node('tuck_in', anonymous=True)

    holding = []

    try:
        if arms == 'r':
            print "Tucking right side"
            hold_side("r", 0.2, holding)
        elif arms == 'l':
            print "Tucking left side"
            hold_side("l", -0.2, holding)
        elif arms == 'b':
            print "Tucking both sides"
            hold_side("l", -0.2, holding)
            hold_side("r", 0.2, holding)
        else:
            print usage
        
        while not rospy.is_shutdown():
            sleep(0.5)
    finally:
        # Kill all holding controllers
        # DOESN'T KILL PROPERLY, NEED TO INVESTIGATE
        print "Releasing controllers"
        for name in holding:
            print "Releasing %s" % name
            for i in range(1,6):
                try:
                    mechanism.kill_controller(name)
                    break # Go to next controller if no exception               
                except:
                    print "Failed to kill controller %s on try %d" % (name, i)

if __name__ == '__main__':
    main()
