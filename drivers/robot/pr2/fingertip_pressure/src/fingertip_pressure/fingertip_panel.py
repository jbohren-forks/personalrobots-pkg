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

PKG = 'fingertip_pressure'

import roslib
roslib.load_manifest(PKG)

import sys
import rospy
from ethercat_hardware.msg import PressureState

from fingertip_pressure.colormap import color
from fingertip_pressure.msg import PressureInfo

import wx
import threading
from wx import xrc

NUMSENSORS = 22
RATE = 5. #Hz 

def txtcolor(data):
    (r,g,b)=color(data)
    if r + g + b < 3 *128:
        (rt,gt,bt) = (255,255,255)
    else:
        (rt,gt,bt) = (0,0,0)
    return ("#%02x%02x%02x"%(r,g,b),"#%02x%02x%02x"%(rt,gt,bt))

class GripperPressurePanel(wx.Panel):
    def __init__(self, parent, topic):
        wx.Panel.__init__(self, parent, wx.ID_ANY)

        # Set up UI
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.panel0 = FingertipPressurePanel(self)
        self.panel1 = FingertipPressurePanel(self)
        sizer.Add(self.panel0.panel, 1, wx.EXPAND)
        sizer.AddSpacer(5)
        sizer.Add(self.panel1.panel, 1, wx.EXPAND)
        self.SetSizer(sizer)

        # Set up subscription
        self._mutex = threading.Lock()
        self.new_message_ = None
        self.new_info_ = None
        rospy.Subscriber(topic, PressureState, self.message_callback)
        rospy.Subscriber(topic+"_info", PressureInfo, self.info_callback)

        self.timer = wx.Timer(self, 1)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)

    def info_callback(self, message):
        self.new_info_ = message

    def message_callback(self, message):
        #print 'message_callback'
        self._mutex.acquire()
        
        self.new_message_ = message
       
        if not self.timer.IsRunning():
            self.timer.Start(1000./RATE, True)
            wx.CallAfter(self.display)
 
        self._mutex.release()

    def on_timer(self, event):
        #print 'timer'
        pass

    def display(self):
        self._mutex.acquire()
        
        if self.new_message_ != None:
            #print 'newmsg'
            
            self.panel0.new_message(self.new_message_.data0)
            self.panel1.new_message(self.new_message_.data1)
            
            self.new_message_ = None
        
            self.Refresh()

        if self.new_info_ != None:
            self.panel0.new_frame(self.new_info_.sensor[0].frame_id)
            self.panel1.new_frame(self.new_info_.sensor[0].frame_id)
            self.new_info_ = None
            self.Refresh()

        self._mutex.release()

class FingertipPressurePanel:
    def __init__(self, parent): 
        xrc_path = roslib.packages.get_pkg_dir(PKG) + '/ui/fingertip_panel.xrc'
        panelxrc = xrc.XmlResource(xrc_path)
            
        self.panel = panelxrc.LoadPanel(parent, 'FingertipPressurePanel')
        
        bag =self.panel.GetSizer()
        bag.SetEmptyCellSize(wx.Size(0,0)) 

        self.pad = []
        for i in range(0, NUMSENSORS):
            self.pad.append(xrc.XRCCTRL(self.panel, 'pad%i'%i))
            self.pad[i].SetEditable(False)
            font = self.pad[i].GetFont()
            font.SetPointSize(6)
            self.pad[i].SetFont(font)
            self.pad[i].SetMinSize(wx.Size(40,30))
        self.frame_id_box = xrc.XRCCTRL(self.panel, 'frame_id')

    def new_message(self, data):
        #print "FingertipPressurePanel new_message"
        for i in range(0, NUMSENSORS):
            #print repr(color(data[i]))
            #print txtcolor(data[i])
            (colb,colf)=txtcolor(data[i])
            self.pad[i].SetBackgroundColour(colb)
            self.pad[i].SetForegroundColour(colf)
            self.pad[i].SetValue('#%i\n%i'%(i,data[i]))
           
    def new_frame(self, frame_id):
        self.frame_id_box.SetValue(frame_id)
