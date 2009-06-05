#!/usr/bin/env python
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

import roslib
roslib.load_manifest('joint_calibration_monitor')

import time
import rospy
import sys
import threading
import math

from diagnostic_msgs.msg import DiagnosticMessage
from roslib import rostime
from joint_calibration_monitor.generic_joint_monitor import *


joint_monitors = [ ]
rx_count = 0

def mech_state_callback(mech_state) :
    global rx_count
    rx_count += 1
    diag_msgs = [x.update(mech_state) for x in joint_monitors]
    
    # Don't publish every single time
    if (rx_count > 10) :
        rx_count = 0
        pub.publish(status=diag_msgs)
        #print "publishing diagnostics"

# Define the flag criteria for PR2's wrist roll joint
def wrist_roll_check(flag, pos) :
    ref = 1.67
    normalized = (pos - ref) % (2*math.pi) - math.pi
    db = 8*math.pi/180              # Deadband around flag for which we don't track errors
    if normalized < -db and normalized > -math.pi + db :
        if flag == 1 :
            return FLAG_OK
        else :
            return FLAG_ERROR
    elif normalized > db and normalized < math.pi - db :
        if flag == 0 :
            return FLAG_OK
        else :
            return FLAG_ERROR
    else :
        return FLAG_DEADBAND


if __name__ == '__main__':

    print 'Starting node'

    rospy.init_node('joint_calibration_monitor', sys.argv, anonymous=False)
    
    # Initialize Each Joint Monitor
    joint_monitors.append(GenericJointMonitor("r_wrist_r_motor",
                                              "r_wrist_roll_joint",
                                              wrist_roll_check,
                                              100) )


    pub = rospy.Publisher("/diagnostics", DiagnosticMessage)
    sub = rospy.Subscriber('mechanism_state', MechanismState,
                           mech_state_callback, None)


    
    rospy.spin()
