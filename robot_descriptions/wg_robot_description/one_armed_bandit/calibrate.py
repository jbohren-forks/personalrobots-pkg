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

rospy.init_node('calibration', anonymous=True)

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
