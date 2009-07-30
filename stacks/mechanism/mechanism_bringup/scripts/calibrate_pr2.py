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
from mechanism_msgs.srv import SpawnController, KillController, SwitchController, SwitchControllerRequest
from imu_node.srv import GetBoolStatus

from robot_mechanism_controllers.srv import *
from robot_mechanism_controllers import controllers


class SendMessageOnSubscribe(rospy.SubscribeListener):
    def __init__(self, msg):
        self.msg = msg
        print "Waiting for subscriber..."

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peer_publish(self.msg)
        sleep(0.1)

spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
kill_controller = rospy.ServiceProxy('kill_controller', KillController)
switch_controller = rospy.ServiceProxy('switch_controller', SwitchController)

controllers_up = []

def slurp(filename):
    f = open(filename)
    stuff = f.read()
    f.close()
    return stuff

def calibrate(joints):
    if rospy.is_shutdown():
        return
    if type(joints) is not list:
        joints = [joints]

    controllers = ["cal_%s" % j for j in joints]
    launched = []
    try:
        # Launches the calibration controllers
        for c in controllers:
            resp = spawn_controller(c)
            if resp.ok == 0:
                print "Failed: %s" % c
            else:
                launched.append(c)
        print "Launched: %s" % ', '.join(launched)

        # Starts the launched controllers
        switch_controller(launched, [], SwitchControllerRequest.BEST_EFFORT)

        # Waits for the calibration controllers to complete
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
            for i in range(5):
                try:
                    kill_controller(name)
                    break # Go to next controller if no exception
                except Exception, ex:
                    print "Failed to kill controller %s on try %d: %s" % (name, i+1, str(ex))

def hold(joint, command):
    controller = "%s_position_controller" % joint
    if controller not in controllers_up:
        for i in range(3):
            try:
                resp = spawn_controller(controller)
                if resp.ok != 0:
                    controllers_up.append(controller)

                    # Starts the launched controllers
                    switch_controller([controller], [], SwitchControllerRequest.BEST_EFFORT)

                    break
            except Exception, ex:
                rospy.logerr("Failed to spawn holding controller %s on try %d: %s" % (controller, i+1, str(ex)))

    rospy.Publisher("%s/set_command" % controller, Float64,
                    SendMessageOnSubscribe(Float64(command)))


def calibrate_imu():
    print "Waiting up to 20s for IMU calibration to complete."
    endtime = rospy.get_time() + 20
    try:
        rospy.wait_for_service('imu/is_calibrated', 20)
        is_calibrated = rospy.ServiceProxy('imu/is_calibrated',GetBoolStatus)
        while True:
            maxtime = max(1,endtime - rospy.get_time())
            if is_calibrated(timeout=maxtime):
                return True
            if rospy.get_time() > endtime:
                return False
            rospy.sleep(1)
    except:
        rospy.logerr("Wait for IMU calibration failed: %s"%sys.exc_info()[0])
        return False

def main():
    rospy.wait_for_service('spawn_controller')

    rospy.init_node('calibration', anonymous=True)

    holding = [] # Tracks which controllers are holding joints by name

    # Calibrate all joints sequentially
    # Hold joints after they're calibrated.
    # Can set desired joint position by using name + /set_command service
    # for joint position controllers

    imustatus = calibrate_imu()
    if not imustatus:
        rospy.logerr("IMU Calibration failed.")

    calibrate('torso_lift')
    hold('torso_lift', 0.08)
    sleep(0.5)

    calibrate('r_shoulder_pan')
    hold('r_shoulder_pan', -0.7)

    calibrate('l_shoulder_pan')
    hold('l_shoulder_pan', 0.7)

    calibrate(['r_elbow_flex', 'l_elbow_flex'])
    hold('r_elbow_flex', -2.0)
    hold('l_elbow_flex', -2.0)

    calibrate(['r_upper_arm_roll', 'l_upper_arm_roll'])
    hold('r_upper_arm_roll', 0.0)
    hold('l_upper_arm_roll', 0.0)

    calibrate(['r_shoulder_lift', 'l_shoulder_lift'])
    hold('r_shoulder_lift', 1.0)
    hold('l_shoulder_lift', 1.0)

    hold('torso_lift', 0.0)

    # Everything else
    calibrate(['r_forearm_roll',
               'l_forearm_roll',
               'r_wrist',
               'r_gripper',
               'l_wrist',
               'l_gripper',
               'laser_tilt',
               'head_pan',
               'head_tilt',
               'caster_fl',
               'caster_fr',
               'caster_bl',
               'caster_br'])


    # Kill all holding controllers
    for name in controllers_up:
        for i in range(5):
            try:
                kill_controller(name)
                break # Go to next controller if no exception
            except:
                rospy.logerr("Failed to kill controller %s on try %d" % (name, i+1))

    if not imustatus:
        rospy.logerr("Mechanism calibration complete, but IMU calibration failed.")
        sys.exit(2)

    rospy.logout("Calibration complete")

if __name__ == '__main__': main()
