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
#* 
#* Author: Alex Sorokin. 
#***********************************************************
import roslib
roslib.load_manifest('annotated_planar_patch_map')
import rospy
import random
from annotated_map_msgs.msg import *

from robot_msgs.msg import *


import os
out_dir='/u/sorokin/ros/ros-pkg/mapping/annotated_planar_patch_map/test_data/'
if not os.path.exists(out_dir):
    os.makedirs(out_dir);


def on_map(map_msg):
    print "Got it"
    outFN='%s/%d.%d.txt' % (out_dir,map_msg.header.stamp.secs,map_msg.header.stamp.nsecs);
    fOut=open(outFN,'w');
    for p in map_msg.polygons:
        for pt in p.points:
            print >>fOut, pt.x,pt.y,pt.z
        print >>fOut, 0,0,0
    fOut.close()

if __name__=="__main__":
    rospy.init_node("dump_ppm", anonymous=True)
    rospy.Subscriber("/planar_map", PolygonalMap, on_map)
    rospy.spin()

