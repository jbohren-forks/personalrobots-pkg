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
import sys, os
from time import sleep

# Loads interface with the robot.
rostools.update_path('mechanism_bringup')
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
        [kill_controller(name) for name in launched]


if __name__ == '__main__':
    rospy.init_node('calibration', anonymous=True)

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

    calibrate(xml)
    print "Calibration complete"
