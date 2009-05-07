#! /usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend

# Publish information needed to interpret fingertep pressure sensor data:
# - Frame id
# - 

import roslib
roslib.load_manifest('fingertip_pressure')
import rospy

from fingertip_pressure.msg import PressureInfo, PressureInfoElement
from robot_msgs.msg import Vector3

force_per_unit_table = [
        ]

# coordinates are in mm here, and get converted to meters for publishing.
coordinates = [
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ( 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ),
        ]

def multorientation(data, ori):
    for i in range(0, len(data)):
        data[i].y = data[i].y * ori
        data[i].z = data[i].z * ori

def extractvec(i):
    out = [];
    for j in range(0,len(coordinates)):
        v = Vector3()
        v.x = coordinates[j][i];
        v.y = coordinates[j][i+1];
        v.z = coordinates[j][i+2];
        out.append(v)
    return out

def pressureInformation(frame_id, orientation):
    msg = PressureInfoElement()
    msg.frame_id = frame_id
    msg.force_per_unit = force_per_unit_table
    msg.center = extractvec(0)
    msg.halfside1 = extractvec(5)
    msg.halfside2 = extractvec(6)
    multorientation(msg.center, orientation)
    multorientation(msg.halfside1, orientation)
    multorientation(msg.center, orientation)
    return msg

def pressureInformationPublisher(topicname, info):
    rospy.init_node('pressure_info', anonymous=True)

    publisher = rospy.Publisher(topicname, PressureInfo)
    
    while not rospy.is_shutdown():
        rospy.sleep(1)
        publisher.publish(info)

if __name__ == '__main__':
    #@todo it would be nice to read an xml configuration file to get these parameters.
    info = []
    info.append(pressureInformation('r_gripper_r_finger_tip_link', 1))
    info.append(pressureInformation('r_gripper_l_finger_tip_link', -1))
    pressureInformationPublisher('pressure/r_gripper_motor_info', info)
