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

import roslib; roslib.load_manifest('mechanism_control')

import sys
import time

import rospy
from mechanism_control import mechanism
from robot_srvs.srv import SpawnController, KillController

# global state of spawned/failed_spawned so that we can teardown controllers on process exit
spawned = []
failed_spawned = []

## kill any controllers that this process has launched
def shutdown():
    kill_controller = rospy.ServiceProxy('kill_controller', KillController)
    for name in reversed(spawned):
        for i in range(3): #try three times to kill
            try:
                rospy.logout("Trying to kill %s" % name)
                kill_controller(name)
                rospy.logout("Succeeded in killing %s" % name)
                break 
            except rospy.ServiceException, se:
                rospy.logerr("ServiceException while killing %s: %s"%(name, se))

## @return [str], [str]: spawned, failed_spawned. Names of controllers
## that successfully loaded and those that failed. Returns None, None if
## controller configs not loaded successfully.
def spawner(configs):
    global spawned, failed_spawned

    # load the controller config xml files
    xmls = []
    for c in configs:
        try:
            f = open(c)
            try:
                xmls.append(f.read())
            finally:
                f.close()
        except IOError, e:
            print >> sys.stderr, "Cannot open controller config file [%s]"%c
            return None, None

    # spawn the configs
    rospy.init_node('spawner', anonymous=True)
    rospy.wait_for_service('spawn_controller')
    rospy.on_shutdown(shutdown)
    spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
    for xml in xmls:
        resp = spawn_controller(xml)
        for ok, name in zip(resp.ok, resp.name):
            if ok == chr(1):
                spawned.append(name)
            else:
                failed_spawned.append(name)
    return spawned, failed_spawned
    
def spawner_main(argv=sys.argv):
    import optparse
    parser = optparse.OptionParser('spawner.py <controllers_config>')
    options, args = parser.parse_args(argv[1:])
    if not args:
        parser.error("you must specify controller config(s) to load")
    spawned, failed_spawned = spawner(args)

    if not spawned:
        sys.exit(-1)
    if failed_spawned:
        print >> sys.stderr, "Failed to spawn: %s" % ', '.join(failed_spawned)
    print "Spawned controllers: %s" % ', '.join(spawned)

    # block until shutdown
    rospy.spin()

if __name__ == '__main__':
    spawner_main(rospy.myargv())
