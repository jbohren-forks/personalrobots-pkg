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

# Reads fingertip pressure data from /pressure and publishes it as a
# visualization_marker

import roslib
roslib.load_manifest('fingertip_pressure')
import rospy

from ethercat_hardware.msg import PressureState
from visualization_msgs.msg import Marker

positions = [ # x, y, z, xscale, yscale, zscale 
        ( 0.026, 0.007, 0.000, 0.010, 0.010, 0.015),
        ( 0.010, 0.010,-0.008, 0.030, 0.010, 0.010), 
        ( 0.025, 0.010,-0.008, 0.010, 0.010, 0.010), 
        ( 0.030, 0.010,-0.004, 0.010, 0.010, 0.010), 
        ( 0.030, 0.010, 0.004, 0.010, 0.010, 0.010), 
        ( 0.025, 0.010, 0.008, 0.010, 0.010, 0.010), 
        ( 0.010, 0.010, 0.008, 0.030, 0.010, 0.010),
        ( 0.021, 0.012,-0.006, 0.010, 0.010, 0.010),
        ( 0.021, 0.012, 0.000, 0.010, 0.010, 0.010),
        ( 0.021, 0.012, 0.006, 0.010, 0.010, 0.010),
        ( 0.016, 0.012,-0.006, 0.010, 0.010, 0.010),
        ( 0.016, 0.012, 0.000, 0.010, 0.010, 0.010),
        ( 0.016, 0.012, 0.006, 0.010, 0.010, 0.010),
        ( 0.011, 0.012,-0.006, 0.010, 0.010, 0.010),
        ( 0.011, 0.012, 0.000, 0.010, 0.010, 0.010),
        ( 0.011, 0.012, 0.006, 0.010, 0.010, 0.010),
        ( 0.006, 0.012,-0.006, 0.010, 0.010, 0.010),
        ( 0.006, 0.012, 0.000, 0.010, 0.010, 0.010),
        ( 0.006, 0.012, 0.006, 0.010, 0.010, 0.010),
        ( 0.001, 0.012,-0.006, 0.010, 0.010, 0.010),
        ( 0.001, 0.012, 0.000, 0.010, 0.010, 0.010),
        ( 0.001, 0.012, 0.006, 0.010, 0.010, 0.010),
        ]
numsensors = len(positions);

def color(data):
    if data < 1000:
        return (0,0,0)
    if data < 3000:
        x = (data-1000)/2000.
        return 0,0,x
    if data < 6000:
        x = (data-3000)/3000.
        return x,0,(1-x)
    if data < 10000:
        x = (data-6000)/4000.
        return 1.0,x,x
    return 1.0,1.0,1.0

class pressureVisualizer:
    def callback(self, pressurestate):
        #print "callback"
        self.data0 = pressurestate.data0
        self.data1 = pressurestate.data1
        self.datatimestamp = pressurestate.header.stamp
        self.dataready = True

    def publish(self):
        if self.dataready:
            #print 'publish'
            self.dataready = False
            self.makeVisualization(self.data0, self.frame0,1)
            self.makeVisualization(self.data1, self.frame1,-1)

    def makeVisualization(self, data, frame, ydir):
        mk = Marker()
        mk.header.frame_id = frame
        mk.header.stamp = self.datatimestamp
        mk.ns = "pressure/" + frame + "/sphere"
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.lifetime = rospy.Duration(1)
        mk.points = []
        for i in range(0,numsensors):
            mk.id = i
            (mk.pose.position.x, mk.pose.position.y, mk.pose.position.z, mk.scale.x, mk.scale.y, mk.scale.z) = positions[i]
            mk.pose.position.y = mk.pose.position.y * ydir
            mk.pose.position.z = mk.pose.position.z * ydir
            mk.pose.orientation.w = 1.0
            mk.color.a = 1.0
            (mk.color.r, mk.color.g, mk.color.b) = color(data[i])
            self.vis_pub.publish(mk)

    def __init__(self, source, frame0, frame1):
        self.frame0 = frame0
        self.frame1 = frame1
        self.dataready = False
        self.source = source

        self.vis_pub = rospy.Publisher('visualization_marker',
                Marker)
        rospy.Subscriber(source, PressureState, self.callback)
        

if __name__ == '__main__':
    #@todo it would be nice to read an xml configuration file to get these parameters.
    rospy.init_node('pressure_visualizer_spheres')
        
    pv1 = pressureVisualizer('pressure/r_gripper_motor', 'r_gripper_r_finger_tip_link', 'r_gripper_l_finger_tip_link')
    pv2 = pressureVisualizer('pressure/l_gripper_motor', 'l_gripper_r_finger_tip_link', 'l_gripper_l_finger_tip_link')
    while not rospy.is_shutdown():
        rospy.sleep(0.02)
        pv1.publish()
        pv2.publish()

