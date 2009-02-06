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

def slurp(filename):
    f = open(filename)
    stuff = f.read()
    f.close()
    return stuff

rospy.wait_for_service('spawn_controller')


def calibrate(config):
    spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
    kill_controller = rospy.ServiceProxy('kill_controller', KillController)

    # Spawns the controllers
    resp = spawn_controller(config)

    # Accumulates the list of spawned controllers
    launched = []
    print "OKs: " + ','.join([str(ok) for ok in resp.ok])
    try:
        for i in range(len(resp.ok)):
            if resp.ok[i] == 0:
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
def xml_for_cal(name, p, i, d, iClamp):
    return '''\
<controller name=\"cal_%s" topic="cal_%s"\
type="JointCalibrationControllerNode">\
<calibrate joint="%s_joint"\
actuator="%s_motor"\
transmission="%s_trans"\
velocity="1.0" />\
<pid p="%d" i="%d" d="%d" iClamp="%d" />\
</controller>''' % (name, name, name, name, name, p, i, d, iClamp)  

def xml_for_hold(name, p, i, d, iClamp):
    return '''\
<controller name=\"%s_controller\" type=\"JointPositionControllerNode\">\
<joint name=\"%s_joint">\
<pid p="%d" i="%d" d="%d" iClamp="%d" />\
</controller>''' % (name, name, p, i, d, iClamp)

def xml_for_wrist(side):
    return '''\
<controller type=\"WristCalibrationControllerNode\" name=\"cal_wrist\">\
<calibrate transmission=\"%s_wrist_trans\"\
actuator_l=\"%s_wrist_l_motor\" actuator_r=\"%s_wrist_r_motor\"\ 
flex_joint=\"%s_wrist_flex_joint\" roll_joint=\"%s_wrist_roll_joint\"\ 
velocity=\"1.2\" />\
<pid p=\"3.0\" i=\"0.2\" d=\"0\" iClamp=\"2.0\" />\
</controller>''' % side

def hold_joint(name, p, i, d, iClamp, holding):
    spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)

    # Try to launch 3x
    # If launched, add to list of holding controllers and return true 
    for i in range(0,2):
        resp = spawn_controller(xml_for_hold(name, 4, 0.5, 0.2, 1.0))
        if resp.ok[0] != 0:
            holding.append(resp.name[0])
            return true

    return false

if __name__ == '__main__':
    rospy.init_node('calibration', anonymous=True)

    spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
    kill_controller = rospy.ServiceProxy('kill_controller', KillController)

    # Set side = r for now
    side = "r"

    holding = [] # Tracks which controllers are being held by name

    # Calibrate all joints sequentially
    # Hold joints after they're calibrated.
    # Can set desired joint position by using name + /set_command service
    # for joint position controllers
    calibrate(xml_for_wrist(r))
    hold_joint(side + "_wrist_roll", 4, 0.5, 0, 1.0, holding)
    hold_joint(side + "_wrist_flex", 4, 0.5, 0, 1.0, holding)

    forearm_roll_name = side + "_forearm_roll"
    calibrate(xml_for_cal(forearm_roll_name, 5, 0, 0, 0))
    hold_joint(forearm_roll_name, 20, 1.0, 0.2, 1.0, holding)

    elbow_flex_name = side + "_elbow_flex"
    calibrate(xml_for_cal(elbow_flex_name, 6, 0.2, 0, 1))
    hold_joint(elbow_flex_name, 100, 20, 10, 2, holding)

    upperarm_roll_name = side + "_upper_arm_roll"
    calibrate(xml_for_cal(upperarm_roll_name, 6, 0.2, 0, 2))
    hold_joint(upperarm_roll_name, 25, 2, 0.5, 1.0, holding)

    shoulder_lift_name = side + "_shoulder_lift"
    calibrate(xml_for_cal(shoulder_lift_name, 9, 1.0, 0, 6))
    hold_joint(shoulder_lift_name, 60, 10, 5, 4, holding)

    shoulder_pan_name = side + "_shoulder_pan"
    calibrate(xml_for_cal(shoulder_pan_name, 7, 0.5, 0, 1.0))
    # Don't bother holding shoulder pan
    
    # Kill all holding controllers
    for name in holding:
        for i in range(1,6):
            try:
                kill_controller(name)
                break # Go to next controller if no exception
            except:
                print "Failed to kill controller %s on try %d" % (name, i)
            

    print "Calibration complete"
