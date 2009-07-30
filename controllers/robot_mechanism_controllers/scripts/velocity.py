#! /usr/bin/python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This script brings up an effort controller on your joint of choice
# and allows you to type in the desired efforts.
#
# Author: Kevin Watts

import random
CONTROLLER_NAME = "quick_velocity_controller_%08d" % random.randint(0,10**8-1)

import sys

import roslib
roslib.load_manifest('robot_mechanism_controllers')
import rospy
from std_msgs.msg import *
from mechanism_control import mechanism
from mechanism_msgs.srv import SpawnController, KillController, SwitchController

def load_joint_config(joint_name,p,i,d,iClamp):
    rospy.set_param(CONTROLLER_NAME+'/type', 'JointVelocityController')
    rospy.set_param(CONTROLLER_NAME+'/joint', joint_name)
    rospy.set_param(CONTROLLER_NAME+'/pid/p', p)
    rospy.set_param(CONTROLLER_NAME+'/pid/i', i)
    rospy.set_param(CONTROLLER_NAME+'/pid/d', d)
    rospy.set_param(CONTROLLER_NAME+'/pid/iClamp', iClamp)


def main():
    # Give suggestions for p, i, d gains for each joint


    if len(sys.argv) < 5:
        print "Usage: velocity.py <joint> <p> <i> <d> <iClamp>"
        sys.exit(1)
    joint = sys.argv[1]
    p = float(sys.argv[2])
    i = float(sys.argv[3])
    d = float(sys.argv[4])
    iClamp = float(sys.argv[5])

    rospy.init_node('velocity', anonymous=True)
    rospy.wait_for_service('spawn_controller')
    rospy.wait_for_service('kill_controller')
    rospy.wait_for_service('switch_controller')
    spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
    kill_controller = rospy.ServiceProxy('kill_controller', KillController)
    switch_controller = rospy.ServiceProxy('switch_controller', SwitchController)

    load_joint_config(joint, p, i, d, iClamp)
    resp = spawn_controller(CONTROLLER_NAME)
    if not resp.ok:
        print "Failed to spawn velocity controller"
        sys.exit(1)
    resp = switch_controller([CONTROLLER_NAME],[],2)
    if not resp.ok:
        print "Failed to start velocity controller"
        sys.exit(1)

    pub = rospy.Publisher("%s/set_command" % CONTROLLER_NAME, Float64)

    try:
        print "Enter velocity:"
        while not rospy.is_shutdown():
            effort = float(sys.stdin.readline().strip())
            pub.publish(Float64(effort))
    finally:
        resp = kill_controller(CONTROLLER_NAME)
        if not resp.ok:
            print "Failed to kill velocity controller"
            sys.exit(1)

if __name__ == '__main__':
    main()
