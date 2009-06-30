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

# Calibrates the PR-2 in a safe sequence

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
    # Spawns the controllers
    resp = spawn_controller(SpawnControllerRequest(config,1))

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
            rospy.Subscriber("%s/calibrated" % name, Empty, calibrated, name)

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
    return """
<controller name="cal_%s" topic="cal_%s" type="JointUDCalibrationControllerNode">
<calibrate joint="%s_joint" actuator="%s_motor"
transmission="%s_trans" velocity="%d" />
<pid p="%d" i="%d" d="%d" iClamp="%d" />
</controller>""" % (name, name, name, name, name, velocity, p, i, d, iClamp)  

def xml_for_hold(name, p, i, d, iClamp):
    return """
<controller name="%s_controller" type="JointPositionControllerNode">
<joint name="%s_joint">
<pid p="%d" i="%d" d="%d" iClamp="%d" />
</controller>""" % (name, name, p, i, d, iClamp)

def xml_for_wrist(side):
    return """
<controller name="cal_%s_wrist"  type="WristCalibrationControllerNode">
<calibrate transmission="%s_wrist_trans"
actuator_l="%s_wrist_l_motor" actuator_r="%s_wrist_r_motor" 
flex_joint="%s_wrist_flex_joint" roll_joint="%s_wrist_roll_joint" 
velocity="1.5" />
<pid p="4.0" i="0.2" d="0" iClamp="2.0" />
</controller>""" % (side, side, side, side, side, side)

def hold_joint(name, p, i, d, iClamp, holding):
    # Try to launch 3x
    # If launched, add to list of holding controllers and return true 
    for i in range(1,4):
        try:
            resp = spawn_controller(xml_for_hold(name, p, i, d, iClamp), 1)
            if resp.ok[0] != 0:
                holding.append(resp.name[0])
                return True
        except:
            rospy.logerr("Failed to spawn holding controller %s on try %d" % (name, i))

    return False

def set_controller(controller, command):
    pub = rospy.Publisher(controller + '/set_command', Float64,
                              SendMessageOnSubscribe(Float64(command)))

def calibrate_imu():
    print "Waiting up to 20s for IMU calibration to complete."
    endtime = rospy.get_time() + 20
    try:
        rospy.wait_for_service('imu/is_calibrated', 20)
        is_calibrated = rospy.ServiceProxy('imu/is_calibrated',GetByte)
        while True:
            maxtime = max(1,endtime - rospy.get_time())
            if is_calibrated(timeout=maxtime):
                return True
            if rospy.get_time() > endtime:
                rospy.logerr("Timed out waiting for IMU calibration.")
                return False
            rospy.sleep(1)
    except:
        rospy.logerr("Wait for IMU calibration failed: %s"%sys.exc_info()[0])
        return False

if __name__ == '__main__':
    rospy.wait_for_service('spawn_controller')

    rospy.init_node('calibration', anonymous=True)

    holding = [] # Tracks which controllers are holding joints by name
    
    # Calibrate all joints sequentially
    # Hold joints after they're calibrated.
    # Can set desired joint position by using name + /set_command service
    # for joint position controllers

    imustatus = calibrate_imu()
    if not imustatus:
        print "IMU Calibration may have failed."
    
    # Check sign on torso lift controller
    calibrate(xml_for_cal("torso_lift", -10, 2000000, 0, 10000, 12000))
    hold_joint("torso_lift", 1000000, 0.0, 10000, 1000000, holding)
    set_controller("torso_lift_controller", float(0.08))
    sleep(0.5)

    calibrate(xml_for_cal("r_shoulder_pan", 1.0, 7, 0.5, 0, 1.0))
    hold_joint("r_shoulder_pan", 60, 10, 5, 4, holding)
    set_controller("r_shoulder_pan_controller", float(-0.7))

    calibrate(xml_for_cal("l_shoulder_pan", 1.0, 7, 0.5, 0, 1.0))
    hold_joint("l_shoulder_pan", 60, 10, 5, 4, holding)
    set_controller("l_shoulder_pan_controller", float(0.7))

    calibrate(xml_for_cal("r_elbow_flex", -1.0, 6, 0.2, 0, 1) + "\n" + xml_for_cal("l_elbow_flex", -1.0, 6, 0.2, 0, 1))
    hold_joint("r_elbow_flex", 100, 20, 10, 2, holding)
    hold_joint("l_elbow_flex", 100, 20, 10, 2, holding)

    set_controller("r_elbow_flex_controller", float(-2.0))
    set_controller("l_elbow_flex_controller", float(-2.0))
    
    upperarm_roll_name = "r_upper_arm_roll"
    calibrate(xml_for_cal(upperarm_roll_name, 1.0, 6, 0.2, 0, 2))
    hold_joint(upperarm_roll_name, 25, 2, 1.0, 1.0, holding)
    set_controller("r_upper_arm_roll_controller", float(0.0))

    upperarm_roll_name = "l_upper_arm_roll"
    calibrate(xml_for_cal(upperarm_roll_name, 1.0, 6, 0.2, 0, 2))
    hold_joint(upperarm_roll_name, 25, 2, 1.0, 1.0, holding)
    set_controller("l_upper_arm_roll_controller", float(0.0))

    shoulder_lift_name = "r_shoulder_lift"
    calibrate(xml_for_cal("r_shoulder_lift", -1.0, 9, 1.0, 0, 6) + "\n" + xml_for_cal("l_shoulder_lift", -1.0, 9, 1.0, 0, 6))

    hold_joint("r_shoulder_lift", 60, 10, 5, 4, holding)
    hold_joint("l_shoulder_lift", 60, 10, 5, 4, holding)
    
    set_controller("r_shoulder_lift_controller", float(1.0))
    set_controller("l_shoulder_lift_controller", float(1.0))

    rospy.logerr('Setting torso lift controller')
    torso_pub = rospy.Publisher('torso_lift_controller/set_command', Float64)
    torso_pub.publish(Float64(float(0.0)))
    sleep(1.5)
    
    # Calibrate other controllers given
    xml = ''
    
    if len(sys.argv) > 1:
        xacro_cmd = roslib.packages.get_pkg_dir('xacro', True) + '/xacro.py'

        xmls = [os.popen2(xacro_cmd + " %s" % f)[1].read() for f in rospy.myargv()[1:]]

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
                    rospy.logerr("Failed to kill controller %s on try %d" % (name, i))
    
    if not imustatus:
        print "Mechanism calibration complete, but IMU calibration failed."
        sys.exit(2)
    
    print "Calibration complete"
