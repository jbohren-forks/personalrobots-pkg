#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rostools
rostools.update_path('cam_viewer_py')

import sys
import time

from std_msgs.msg import Image
from std_msgs.msg import ImageArray
import rospy
import pyrob.util as ut
import opencv.highgui as hg

def display_array(iar):
    left  = ut.ros2cv(iar.images[0])
    right = ut.ros2cv(iar.images[1])

    hg.cvShowImage('channel 1', left)
    hg.cvShowImage('channel 2', right)
    hg.cvWaitKey(5)

def display_single(im):
    imcv = ut.ros2cv(im)
    hg.cvShowImage('channel 1', imcv)
    hg.cvWaitKey(5)

if __name__ == '__main__':
    hg.cvNamedWindow('channel 1', 1)
    hg.cvNamedWindow('channel 2', 1)
    rospy.Subscriber('images', ImageArray, display_array)
    rospy.Subscriber('image', Image, display_single)
    rospy.init_node('camview_py')
    rospy.spin()

