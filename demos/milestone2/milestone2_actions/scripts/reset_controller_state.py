#! /usr/bin/env python
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

# Brings up a set of controllers when run, and brings them down when
# killed.  Extremely useful for spawning a set of controllers from
# roslaunch.
#
# Author: Wim Meeussen

import roslib, time
roslib.load_manifest('milestone2_actions')

import rospy, sys
from mechanism_control import mechanism
from robot_srvs.srv import SwitchController

def print_usage(exit_code = 0):
    print 'reset_controller_state.py <controller1 controller2 ...>'
    sys.exit(exit_code)


def stop_controller(controller):
    try:
        print "Stopping controller"
        switcher = rospy.ServiceProxy('switch_controller', SwitchController)
        switcher([], [controller])
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage()
    rospy.init_node('reset_controller_state', anonymous=True)

    rospy.wait_for_service('switch_controller')
    switch_controller = rospy.ServiceProxy('switch_controller', SwitchController)

    for c in range(1,len(sys.argv)):
         stop_controller(sys.argv[c])
