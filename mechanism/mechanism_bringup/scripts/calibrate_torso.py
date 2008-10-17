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

# Author: Stuart Glaser

import rostools
import copy
import threading
from time import sleep

# Loads interface with the robot.
rostools.update_path('mechanism_bringup')

import rospy
from std_msgs.msg import Empty
from robot_srvs.srv import *
from robot_mechanism_controllers.srv import *

def slurp(filename):
    f = open(filename)
    stuff = f.read()
    f.close()
    return stuff

rospy.wait_for_service('spawn_controller')
spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
kill_controller = rospy.ServiceProxy('kill_controller', KillController)


# Hits the joint stops
def calibrate_blindly(config):
    resp = spawn_controller(config)
    if len(resp.ok) != 1 or not resp.ok[0]:
        print "FAIL"
        return
    name = resp.name[0]
    try:
        do_calibration = rospy.ServiceProxy("/%s/calibrate" % name, CalibrateJoint)
        do_calibration()
    finally:
        kill_controller(name)
    print "Calibrated"



def calibrate(config):
    # Spawns the controllers
    resp = spawn_controller(config)

    # Accumulates the list of spawned controllers
    launched = []
    try:
        for i in range(len(resp.ok)):
            if not resp.ok[i]:
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
        while waiting_for:
            print "Waiting for: %s" % ', '.join(waiting_for)
            sleep(0.5)
    finally:
        [kill_controller(name) for name in launched]


rospy.init_node('calibration', anonymous=True)



calibrate('''
  <controller type="JointCalibrationControllerNode" name="cal_torso" topic="cal_torso">
    <calibrate joint="torso_joint"
               actuator="torso_motor"
               transmission="torso_trans"
               velocity="9.0" />
    <pid p="1.0" i="0.4" d="0" iClamp="2" />
  </controller>
''')

print "Calibrated"
