#!/usr/bin/env python

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

import rospy
from sound_play.msg import SoundRequest

class SoundHandle:
    def __init__(self):
        self.pub = rospy.Publisher('robotsound', SoundRequest)

    def say(self,text):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_ONCE
        msg.arg=text
        self.pub.publish(msg)

    def repeat(self,text):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_START
        msg.arg=text
        self.pub.publish(msg)

    def stopsaying(self,text):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_STOP
        msg.arg=text
        self.pub.publish(msg)

    def playwave(self,sound):
        msg = SoundRequest()
        msg.sound = SoundRequest.PLAY_FILE
        msg.command = SoundRequest.PLAY_ONCE
        msg.arg=sound
        self.pub.publish(msg)

    def startwave(self,sound):
        msg = SoundRequest()
        msg.sound = SoundRequest.PLAY_FILE
        msg.command = SoundRequest.PLAY_START
        msg.arg=sound
        self.pub.publish(msg)

    def stopwave(self,sound):
        msg = SoundRequest()
        msg.sound = SoundRequest.PLAY_FILE
        msg.command = SoundRequest.PLAY_STOP
        msg.arg=sound
        self.pub.publish(msg)

    def play(self,sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_ONCE
        self.pub.publish(msg)

    def start(self,sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_START
        self.pub.publish(msg)

    def stop(self,sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_STOP
        self.pub.publish(msg)

    def stopall(self):
        self.stop(SoundRequest.ALL)

