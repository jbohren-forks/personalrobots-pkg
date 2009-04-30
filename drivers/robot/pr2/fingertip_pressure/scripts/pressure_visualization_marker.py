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
# visualizationMarker

import roslib
roslib.load_manifest('fingertip_pressure')
import rospy

from ethercat_hardware.msg import PressureState
from visualization_msgs.msg import VisualizationMarker

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
        return 0,0,255*x
    if data < 6000:
        x = (data-3000)/3000.
        return 255*x,0,255*(1-x)
    if data < 10000:
        x = (data-6000)/4000.
        return 255,255*x,255*x
    return 255,255,255

class pressureVisualizer:
    def callback(self, pressurestate):
        #print "callback"
        self.data0 = pressurestate.data0
        self.data1 = pressurestate.data1
        self.dataready = True

    def publish(self):
        if self.dataready:
            #print 'publish'
            self.dataready = False
            self.makeVisualization(self.data0, self.frame0, self.startid,1)
            self.makeVisualization(self.data1, self.frame1, self.startid + numsensors,-1)

    def makeVisualization(self, data, frame, sensorstartid, ydir):
        mk = VisualizationMarker()
        mk.header.frame_id = frame
        # @todo Make timestamp come from when data is collected in hardware
        mk.header.stamp = rospy.get_rostime() 
        mk.type = VisualizationMarker.SPHERE
        mk.action = VisualizationMarker.ADD
        mk.points = []
        for i in range(0,numsensors):
            mk.id = sensorstartid + i
            (mk.x, mk.y, mk.z, mk.xScale, mk.yScale, mk.zScale) = positions[i]
            mk.y = mk.y * ydir
            mk.z = mk.z * ydir
            mk.roll = mk.pitch = mk.yaw = 0
            mk.alpha = 255
            (mk.r, mk.g, mk.b) = color(data[i])
            self.vis_pub.publish(mk)

    def __init__(self, source, frame0, frame1, startid):
        self.startid = startid
        self.frame0 = frame0
        self.frame1 = frame1
        self.dataready = False

        rospy.init_node('pressureVisualizer', anonymous=True)
        
        self.vis_pub = rospy.Publisher('visualizationMarker',
                VisualizationMarker)
        rospy.Subscriber(source, PressureState, self.callback)
        
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.publish()


if __name__ == '__main__':
    #@todo it would be nice to read an xml configuration file to get these parameters.
    pressureVisualizer('pressure/r_gripper_motor', 'r_gripper_r_finger_tip_link',
            'r_gripper_l_finger_tip_link', 0)
    pressureVisualizer('pressure/l_gripper_motor', 'l_gripper_r_finger_tip_link',
            'l_gripper_l_finger_tip_link', 22)
