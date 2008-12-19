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

import numpy as np
import opencv as cv
import opencv.highgui as hg
import Image as PImage

from std_msgs.msg import Image
from std_msgs.msg import ImageArray
import rospy
import pyrob.util

#def callback_array(iar):
#    left = iar.images[0]
#    right = iar.images[1]
#
#    start   = time.time()
#    leftcv  = to_cv(left)
#    rightcv = to_cv(right)
#    print '%.4f' % (time.time() - start)
#
#    print iar.header.seq,
#    hg.cvShowImage('left', leftcv)
#    hg.cvShowImage('right', rightcv)
#    hg.cvWaitKey(5)

def callback_image(im):
    t = time.time()
    cvim = pyrob.util.ros2cv(im)
    hg.cvShowImage('left', cvim)
    hg.cvWaitKey(5)
    print 'total', time.time() - t

if __name__ == '__main__':
    hg.cvNamedWindow('left', 1)
    rospy.Subscriber('image', Image, callback_image)
    rospy.init_node('test_pycv')
    rospy.spin()

