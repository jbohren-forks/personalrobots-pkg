
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

PKG = 'ocean_battery_driver'

import rostools
rostools.update_path(PKG)

import sys
import rospy
from robot_msgs.msg import *


import wx
from wx import xrc

import threading, time

class BatteryPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent, wx.ID_ANY)
        
        self._mutex = threading.Lock()
        
        xrc_path = rostools.packspec.get_pkg_dir(PKG) + '/ui/battery_status_panel.xrc'
        
        self._xrc = xrc.XmlResource(xrc_path)
        self._real_panel = self._xrc.LoadPanel(self, 'BatteryStatusPanel')
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self._real_panel, 1, wx.EXPAND)
        self.SetSizer(sizer)

        rospy.Subscriber("battery_state", BatteryState, self.message_callback)
        
        self.power_text = xrc.XRCCTRL(self._real_panel, 'm_powerField')
        self.energy_text = xrc.XRCCTRL(self._real_panel, 'm_EnergyField')
        self.status_text = xrc.XRCCTRL(self._real_panel, 'm_statusField')

        self.power_text.SetEditable(False)
        self.energy_text.SetEditable(False)
        self.status_text.SetEditable(False)

        self._messages = []
        
    def message_callback(self, message):
        self._mutex.acquire()
        
        self._messages.append(message)
        
        self._mutex.release()
        
        wx.CallAfter(self.new_message)
        
    def new_message(self):


        self._mutex.acquire()
        for message in self._messages:
            ratio = message.energy_remaining / max(message.energy_capacity, 0.0001)
            self.power_text.SetValue('%.2f Watts'%message.power_consumption)
            self.energy_text.SetValue('%.2f of %.2f Joules    %.1f Percent'%(message.energy_remaining, message.energy_capacity, ratio*100.0 ))
            if ratio > 0.7:
                self.energy_text.SetBackgroundColour("Light Green")
            elif ratio > 0.3:
                self.energy_text.SetBackgroundColour("Orange")
            else:
                self.energy_text.SetBackgroundColour("Red")
                                
            
        

##        self.textboxes[0].value = "hi"       
        self._messages = []
        
        self._mutex.release()
        
        self.Refresh()
        
