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
import sys
from time import sleep

# Loads interface with the robot.
rostools.update_path('teleop_robot')
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
spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
kill_controller = rospy.ServiceProxy('kill_controller', KillController)


def calibrate_optically(config):
    resp = spawn_controller(config)
    if len(resp.ok) != 1 or not resp.ok[0]:
        print "FAIL"
        return
    name = resp.name[0]
    do_calibration = rospy.ServiceProxy("/%s/calibrate" % name, CalibrateJoint)
    do_calibration()
    kill_controller(name)
    print "Calibrated"

def calibrate_manually(config):
    resp = spawn_controller(config)
    if len(resp.ok) != 1 or not resp.ok[0]:
        print "FAIL"
        return
    name = resp.name[0]
    begin = rospy.ServiceProxy("/%s/begin_manual_calibration" % name, CalibrateJoint)
    end = rospy.ServiceProxy("/%s/end_manual_calibration" % name, CalibrateJoint)
    begin()
    print "Move the joint to the limits, and then hit enter"
    raw_input()
    end()
    kill_controller(name)
    print "Calibrated manually"

# Hits the joint stops
def calibrate_blindly(config):
    resp = spawn_controller(config)
    if len(resp.ok) != 1 or not resp.ok[0]:
        print "FAIL"
        return
    name = resp.name[0]
    do_calibration = rospy.ServiceProxy("/%s/calibrate" % name, CalibrateJoint)
    do_calibration()
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
        while waiting_for and not rospy.is_shutdown():
            print "Waiting for: %s" % ', '.join(waiting_for)
            sleep(0.5)
    finally:
        [kill_controller(name) for name in launched]


rospy.init_node('calibration', anonymous=True)


calibrate('''
<controllers>

  <controller name="cal_shoulder_pan" topic="cal_shoulder_pan" type="JointCalibrationControllerNode">
    <calibrate joint="shoulder_pan_right_joint"
               actuator="shoulder_pan_right_motor"
               transmission="shoulder_pan_right_trans"
               velocity="0.6" />
    <pid p="7" i="0.5" d="0" iClamp="1.0" />
  </controller>

  <controller name="cal_shoulder_pitch" topic="cal_shoulder_pitch" type="JointCalibrationControllerNode">
    <calibrate joint="shoulder_pitch_right_joint"
               actuator="shoulder_pitch_right_motor"
               transmission="shoulder_pitch_right_trans"
               velocity="0.6" />
    <pid p="7" i="0.5" d="0" iClamp="1.0" />
  </controller>

  <controller name="cal_laser_tilt" topic="cal_laser_tilt" type="JointCalibrationControllerNode">
    <calibrate joint="tilt_laser_mount_joint"
               actuator="tilt_laser_motor"
               transmission="tilt_laser_mount_trans"
               velocity="-0.6" />
    <pid p=".25" i="0.1" d="0" iClamp="1.0" />
  </controller>

</controllers>
''')


calibrate_blindly('''
<controller name="upperarm_calibration" topic="upperarm_calibration" type="JointBlindCalibrationControllerNode">
  <calibrate joint="upperarm_roll_right_joint"
             actuator="upperarm_roll_right_motor"
             transmission="upperarm_roll_right_trans"
             velocity="1.3" />
  <pid p="5" i="0.5" d="0" iClamp="1.0" />
</controller>

''')

calibrate_blindly('''
<controller name="cal_elbow" topic="cal_elbow" type="JointBlindCalibrationControllerNode">
  <calibrate joint="elbow_flex_right_joint"
             actuator="elbow_flex_right_motor"
             transmission="elbow_flex_right_trans"
             velocity="1.0" />
  <pid p="5" i="0.5" d="0" iClamp="1.0" />
</controller>

''')

print "Calibration complete"
