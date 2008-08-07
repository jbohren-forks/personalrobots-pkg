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
#

## Gazebo test cameras validation 

PKG = 'rosgazebo'
NAME = 'testcameras'

import rostools
rostools.update_path(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *

FRAME_TARGET = "genericCam-0050.jpg"
FRAME_DIR = "testcameraframes"

class PollCameraThread(threading.Thread):
    def __init__(self, target, dir):
        super(PollCameraThread, self).__init__()
        self.target = target
        self.dir = dir
    
    def run(self):
        while 1:
            time.sleep(0.01)
            if (os.path.isdir(self.dir)):
                ls = os.listdir(self.dir)
                #print "Dir: " + str(ls)
                for file in ls:
                    if (file == FRAME_TARGET):
                        self.target.onTargetFrame()
                        return

class TestCameras(unittest.TestCase):
    def __init__(self, *args):
        super(TestCameras, self).__init__(*args)
        self.success = False
        self.pollThread = PollCameraThread(self, FRAME_DIR)

    def onTargetFrame(self):
        time.sleep(0.5) #Safety, to make sure the image is really done being written.
        ps = "diff -q " + FRAME_DIR + "/" + FRAME_TARGET + " test/testcamera.valid.jpg"
        #print "CMD: " + ps + "\n"
        result = os.system(ps)
        if (result == 0):
            self.success = True
            #print "Success\n"
        else:
            self.success = False
            #print "Failure\n"
        os.system("killall gazebo") #Huge temp hack to kill gazebo.
        rospy.signal_shutdown('test done')
        
    def test_cameras(self):
        rospy.ready(NAME, anonymous=True)
        self.pollThread.start()
        timeout_t = time.time() + 30.0 #30 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success)
        
    


if __name__ == '__main__':
    while (os.path.isfile(FRAME_DIR + "/" + FRAME_TARGET)):
        print "Old test case still alive."
        time.sleep(0.00001)
    
    rostest.run(PKG, sys.argv[0], TestCameras, sys.argv) #, text_mode=True)


