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

# Calibrate the arm sequentially
# Wrist -> Elbow roll -> Elbow flex -> Upperarm Roll -> 
# Shoulder Lift -> Shoulder Pan

import roslib
import copy
import threading
import sys, os
from time import sleep

# Loads interface with the robot.
roslib.load_manifest('mechanism_bringup')
import rospy
from std_msgs.msg import *
from robot_srvs.srv import *

from robot_mechanism_controllers.srv import *
from robot_mechanism_controllers import controllers


class SendMessageOnSubscribe(rospy.SubscribeListener):
    def __init__(self, msg):
        self.msg = msg
        print "Waiting for subscriber..."

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peer_publish(self.msg)
        sleep(0.1)  
        # TODO: change this line when flushing messages is implemented                                                                             
      #  rospy.signal_shutdown("Done")
       # sys.exit(0)


spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
kill_controller = rospy.ServiceProxy('kill_controller', KillController)

def slurp(filename):
    f = open(filename)
    stuff = f.read()
    f.close()
    return stuff

def calibrate(config):
    #spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
    #kill_controller = rospy.ServiceProxy('kill_controller', KillController)

    # Spawns the controllers
    resp = spawn_controller(config)

    # Accumulates the list of spawned controllers
    launched = []
    print "OKs: " + ','.join([str(ord(ok)) for ok in resp.ok])
    try:
        for i in range(len(resp.ok)):
            if ord(resp.ok[i]) == 0:
                print "Failed: %s" % resp.name[i]
            else:
                launched.append(resp.name[i])
        print "Launched: %s" % ', '.join(launched)

        # Sets up callbacks for calibration completion
        waiting_for = launched[:]
        def calibrated(msg, name):  # Somewhat not thread-safe
            if name in waiting_for:
                waiting_for.remove(name)
        for name in waiting_for:
            rospy.Subscriber("/%s/calibrated" % name, Empty, calibrated, name)

        # Waits until all the controllers have calibrated
        while waiting_for and not rospy.is_shutdown():
            print "Waiting for: %s" % ', '.join(waiting_for)
            sleep(0.5)
    finally:
        # Try to kill controllers several times
        # Make sure they're dead.
        for name in launched:
            for i in range(1,6):
                try:
                    kill_controller(name)
                    break # Go to next controller if no exception
                except:
                    print "Failed to kill controller %s on try %d" % (name, i)
            
#
# Functions make xml code for controllers 
#
def xml_for_cal(name, velocity, p, i, d, iClamp):
    return '''\
<controller name=\"cal_%s" topic="cal_%s"\
type="JointCalibrationControllerNode">\
<calibrate joint="%s_joint"\
actuator="%s_motor"\
transmission="%s_trans"\
velocity="%d" />\
<pid p="%d" i="%d" d="%d" iClamp="%d" />\
</controller>''' % (name, name, name, name, name, velocity, p, i, d, iClamp)  

def xml_for_hold(name, p, i, d, iClamp):
    return '''\
<controller name=\"%s_controller\" type=\"JointPositionControllerNode\">\
<joint name=\"%s_joint">\
<pid p="%d" i="%d" d="%d" iClamp="%d" />\
</controller>''' % (name, name, p, i, d, iClamp)

def xml_for_wrist(side):
    return '''\
<controller name=\"cal_%s_wrist\"  type=\"WristCalibrationControllerNode\" >\
<calibrate transmission=\"%s_wrist_trans\"\
actuator_l=\"%s_wrist_l_motor\" actuator_r=\"%s_wrist_r_motor\"\ 
flex_joint=\"%s_wrist_flex_joint\" roll_joint=\"%s_wrist_roll_joint\"\ 
velocity=\"1.5\" />\
<pid p=\"4.0\" i=\"0.2\" d=\"0\" iClamp=\"2.0\" />\
</controller>''' % (side, side, side, side, side, side)

def hold_joint(name, p, i, d, iClamp, holding):
    # Try to launch 3x
    # If launched, add to list of holding controllers and return true 
    for i in range(1,4):
        try:
            resp = spawn_controller(xml_for_hold(name, 4, 0.5, 0.2, 1.0))
            if resp.ok[0] != 0:
                holding.append(resp.name[0])
                return True
        except:
            print "Failed to spawn holding controller %s on try %d" % (name, i)

    return False

def set_controller(controller, command):
    #    try:
    #rospy.init_node('control', anonymous = True)
    #finally:
    pub = rospy.Publisher('/' + controller + '/set_command', Float64,
                              SendMessageOnSubscribe(Float64(command)))


if __name__ == '__main__':
    rospy.wait_for_service('spawn_controller')
    #   if  rospy.is_shutdown(): 
    #    return

    rospy.init_node('calibration', anonymous=True)

    holding = [] # Tracks which controllers are holding joints by name
    
    # Calibrate all joints sequentially
    # Hold joints after they're calibrated.
    # Can set desired joint position by using name + /set_command service
    # for joint position controllers
    
    # Wrist and forearm calibrated with forearm_calibrator in arm_defs.xml
        
    print "Calibrating elbow flex"
    calibrate(xml_for_cal("r_elbow_flex", -1.0, 6, 0.2, 0, 1) + "\n" + xml_for_cal("l_elbow_flex", -1.0, 6, 0.2, 0, 1))
    hold_joint("r_elbow_flex", 100, 20, 10, 2, holding)
    hold_joint("l_elbow_flex", 100, 20, 10, 2, holding)

    set_controller("r_elbow_flex_controller", float(3.0))
    set_controller("l_elbow_flex_controller", float(3.0))
    
    print "Calibrating right upperarm roll"
    upperarm_roll_name = "r_upper_arm_roll"
    calibrate(xml_for_cal(upperarm_roll_name, 1.0, 6, 0.2, 0, 2))
    hold_joint(upperarm_roll_name, 25, 2, 1.0, 1.0, holding)

    print "Calibrating left upperarm roll" 
    upperarm_roll_name = "l_upper_arm_roll"
    calibrate(xml_for_cal(upperarm_roll_name, 1.0, 6, 0.2, 0, 2))
    hold_joint(upperarm_roll_name, 25, 2, 1.0, 1.0, holding)
        
    print "Calibrating shoulder lift"
    shoulder_lift_name = "r_shoulder_lift"
    calibrate(xml_for_cal("r_shoulder_lift", -1.0, 9, 1.0, 0, 6) + "\n" + xml_for_cal("l_shoulder_lift", -1.0, 9, 1.0, 0, 6))


    hold_joint("r_shoulder_lift", 60, 10, 5, 4, holding)
    hold_joint("l_shoulder_lift", 60, 10, 5, 4, holding)
    
    set_controller("r_shoulder_lift_controller", float(3.0))
    set_controller("l_shoulder_lift_controller", float(3.0))

    print "Calibrating shoulder pan" 
    calibrate(xml_for_cal("r_shoulder_pan", 1.0, 7, 0.5, 0, 1.0))
    hold_joint("r_shoulder_pan", 60, 10, 5, 4, holding)
    set_controller("r_shoulder_pan_controller", float(-0.5))

    calibrate(xml_for_cal("l_shoulder_pan", 1.0, 7, 0.5, 0, 1.0))
    hold_joint("l_shoulder_pan", 60, 10, 5, 4, holding)
    set_controller("l_shoulder_pan_controller", float(0.5))

    sleep(0.5)
    print "Calibrating rest of robot"

    # Calibrate other controllers given
    xml = ''
    
    if len(sys.argv) > 1:
        #xmls = [slurp(filename) for filename in sys.argv[1:]]
        xmls = [os.popen2("rosrun xacro xacro.py %s" % f)[1].read() for f in sys.argv[1:]]

        # Poor man's xml splicer
        for i in range(len(xmls) - 1):
            xmls[i] = xmls[i].replace('</controllers>', '')
            xmls[i+1] = xmls[i+1].replace('<controllers>', '')
        xml = "\n".join(xmls)
    else:
        print "Reading from stdin..."
        xml = sys.stdin.read()

    try:
        calibrate(xml)
    finally:
        # Kill all holding controllers
        for name in holding:
            for i in range(1,6):
                try:
                    kill_controller(name)
                    break # Go to next controller if no exception
                except:
                    print "Failed to kill controller %s on try %d" % (name, i)

    print "Calibration complete"
