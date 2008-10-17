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

import rostools, time
rostools.update_path('mechanism_control')

import rospy, sys
from mechanism_control import mechanism
from robot_srvs.srv import SpawnController, KillController

def print_usage(exit_code = 0):
    print 'spawner.py <controllers_config>'
    sys.exit(exit_code)

rospy.wait_for_service('spawn_controller')
spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
kill_controller = rospy.ServiceProxy('kill_controller', KillController)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage()
    rospy.init_node('spawner', anonymous=True)

    f = open(sys.argv[1])
    xml = f.read()
    f.close()

    resp = spawn_controller(xml)

    spawned = []
    for i in range(len(resp.ok)):
        if resp.ok[i]:
            spawned.append(resp.name[i])
        else:
            print "Failed to spawn %s" % resp.name[i]
    print "Spawned controllers: %s" % ', '.join(spawned)

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    finally:
        [kill_controller(name) for name in spawned]
