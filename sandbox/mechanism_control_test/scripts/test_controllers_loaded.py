#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
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

# Tests that controllers are loaded.  Fails if any of the controllers on the
# argument list have not loaded after no new controllers are loaded for 10
# seconds.

# Author: Stuart Glaser

STABLIZE_TIME = 10.0

import roslib; roslib.load_manifest('mechanism_control_test')
import sys, time
import unittest
import rospy
from mechanism_msgs.srv import *
from mechanism_control import mechanism
from mechanism_msgs.srv import *

rospy.wait_for_service('list_controllers')
list_controllers_srv = rospy.ServiceProxy('list_controllers', ListControllers)
def list_controllers():
    resp = list_controllers_srv().controllers
    # HACK until #2654 is fixed
    controllers = []
    for c in resp:
        i = c.index(' ')
        controllers.append(c[:i])
    return controllers

#resp = s.call(ListControllersRequest())


class TestControllersLoaded(unittest.TestCase):
    def test_controllers_loaded(self):
        #controllers_expected = rospy.myargv()[1:]
        controllers_expected = [c for c in rospy.myargv()[1:] if not c.startswith('-')]
        controllers_up = list_controllers()

        count = 0
        while not rospy.is_shutdown():
            time.sleep(STABLIZE_TIME)
            controllers_up_latest = list_controllers()
            controllers_up_latest.sort()
            print ".",
            if controllers_up_latest == controllers_up:
                break

            controllers_up = controllers_up_latest

            count += 1
            if count > 100:
                self.fail("The loaded controllers won't stabilize!  Giving up")
                return
        print

        print "Expected: ", ', '.join(controllers_expected)
        print "Up: ", ', '.join(controllers_up)
        for c in controllers_expected:
            self.assert_(c in controllers_up, "Expected controller %s to be loaded, but it wasn't" % c)


if __name__ == '__main__':
    import rostest
    rostest.rosrun("test_controller_loaded", "test_controllers_loaded", TestControllersLoaded)
