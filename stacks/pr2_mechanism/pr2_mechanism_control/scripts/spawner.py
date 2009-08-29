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
# Author: Stuart Glaser

import roslib, time
roslib.load_manifest('pr2_mechanism_control')

import rospy, sys
import os.path
from pr2_mechanism_control import mechanism
from pr2_mechanism_msgs.srv import SpawnController, KillController, SwitchController

from xml.dom.minidom import parse, parseString
import xml.dom
import signal

def print_usage(exit_code = 0):
    print 'spawner.py [--stopped] <controller names>'
    sys.exit(exit_code)

rospy.wait_for_service('spawn_controller')
spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
kill_controller = rospy.ServiceProxy('kill_controller', KillController)
switch_controller = rospy.ServiceProxy('switch_controller', SwitchController)

spawned = []
prev_handler = None


def shutdown(sig, stackframe):
    global spawned
    for name in reversed(spawned):
        for i in range(3):
            try:
                rospy.logout("Trying to kill %s" % name)
                kill_controller(name)
                rospy.logout("Succeeded in killing %s" % name)
                break
            except rospy.ServiceException:
                raise
                rospy.logerr("ServiceException while killing %s" % name)
    # We're shutdown.  Now invoke rospy's handler for full cleanup.
    if prev_handler is not None:
        prev_handler(signal.SIGINT,None)

if __name__ == '__main__':
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
    rospy.init_node('spawner', anonymous=True)

    autostart = 1
    argstart = 1
    if argv[1] == '--stopped':
        autostart = 0
        argstart = 2

    # Override rospy's signal handling.  We'll invoke rospy's handler after
    # we're done shutting down.
    prev_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, shutdown)

    for c in range(argstart,len(argv)):
        name = argv[c]
        if os.path.exists(name):
            # Spawning with XML, put here for backwards compatibility
            # so all out launch scripts don't break.
            f = open(name)
            doc = parse(f)
            f.close()

            c_node = doc.firstChild
            if c_node.tagName == 'controllers':
                c_node = c_node.firstChild
            while c_node:
                if c_node.nodeType == xml.dom.Node.ELEMENT_NODE and c_node.tagName == 'controller':
                    n = str(c_node.getAttribute('name'))
                    type = str(c_node.getAttribute('type'))
                    rospy.set_param(n + "/xml", c_node.toxml())
                    rospy.set_param(n + "/type", type)
                    resp = spawn_controller(n)
                    if resp.ok != 0:
                        spawned.append(n)
                    else:
                        time.sleep(1) # give error message a chance to get out
                        rospy.logerr("Failed to spawn %s" % n)
                c_node = c_node.nextSibling
        else:
            resp = spawn_controller(name)
            if resp.ok != 0:
                spawned.append(name)
            else:
                time.sleep(1) # give error message a chance to get out
                rospy.logerr("Failed to spawn %s" % name)

    rospy.loginfo("Spawned controllers: %s" % ', '.join(spawned))

    # start controllers is requested
    if autostart:
        resp = switch_controller(spawned, [], 2)
        if resp.ok != 0:
            rospy.loginfo("Started controllers: %s" % ', '.join(spawned))
        else:
            rospy.logerr("Failed to start controllers: %s" % ', '.join(spawned))

    rospy.spin()
