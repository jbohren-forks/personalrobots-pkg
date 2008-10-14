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

PKG = 'pr2_power_board'

import rostools
rostools.update_path(PKG)

import sys
import rospy
from robot_msgs.msg import *
from pr2_power_board.srv import *

import wx
from wx import xrc

import threading, time

class PowerBoardPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent, wx.ID_ANY)
        
        self._mutex = threading.Lock()
        
        xrc_path = rostools.packspec.get_pkg_dir(PKG) + '/pr2_power_board.xrc'
        
        self._xrc = xrc.XmlResource(xrc_path)
        self._real_panel = self._xrc.LoadPanel(self, 'PowerBoardPanel')
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self._real_panel, 1, wx.EXPAND)
        self.SetSizer(sizer)

        self._real_panel.Bind(wx.EVT_BUTTON, self.EnableCB0, id=xrc.XRCID('m_button1'))
        self._real_panel.Bind(wx.EVT_BUTTON, self.EnableCB1, id=xrc.XRCID('m_button2'))
        self._real_panel.Bind(wx.EVT_BUTTON, self.EnableCB2, id=xrc.XRCID('m_button3'))

        self._real_panel.Bind(wx.EVT_BUTTON, self.StandbyCB0, id=xrc.XRCID('m_button11'))
        self._real_panel.Bind(wx.EVT_BUTTON, self.StandbyCB1, id=xrc.XRCID('m_button21'))
        self._real_panel.Bind(wx.EVT_BUTTON, self.StandbyCB2, id=xrc.XRCID('m_button31'))

        self._real_panel.Bind(wx.EVT_BUTTON, self.DisableCB0, id=xrc.XRCID('m_button12'))
        self._real_panel.Bind(wx.EVT_BUTTON, self.DisableCB1, id=xrc.XRCID('m_button22'))
        self._real_panel.Bind(wx.EVT_BUTTON, self.DisableCB2, id=xrc.XRCID('m_button32'))

        rospy.Subscriber("/diagnostics", DiagnosticMessage, self.diagnostics_callback)
        
        self.power_control = rospy.ServiceProxy('power_board_control', PowerBoardCommand)


        self.voltages = [0,0,0]
        self.breaker_state = ["", "", ""]
#fixme        self.textboxes = [xrc.XRCCTRL(self._xrc, 'm_textCtrl1'), 0, 0]
#        self.textboxes[0].value = "hi"

        self._messages = []
        
    def diagnostics_callback(self, message):
        self._mutex.acquire()
        
        self._messages.append(message)
        
        self._mutex.release()
        
        wx.CallAfter(self.new_message)
        
    def new_message(self):


        self._mutex.acquire()
        

        for message in self._messages:
            for status in message.status:
                if (status.name == "Power board"):
                    for value in status.values:
                        if (value.label == "Breaker 0 Voltage"):
                            self.voltage[0] = value.value
                        if (value.label == "Breaker 1 Voltage"):
                            self.voltage[1] = value.value
                        if (value.label == "Breaker 2 Voltage"):
                            self.voltage[2] = value.value
                    for strvals in status.strings:
                        if (strvals.label == "Breaker 0 State"):
                            self.breaker_state[0] = value.value
                        if (strvals.label == "Breaker 1 State"):
                            self.breaker_state[1] = value.value
                        if (strvals.label == "Breaker 2 State"):
                            self.breaker_state[2] = value.value

        self.textboxes[0].value = "hi"
        
        self._messages = []
        
        self._mutex.release()
        
        self.Refresh()
        
    def EnableCB0(self, event):
        try:
            self.power_control(0, "start")
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
        print "Enable CB0"
    def EnableCB1(self, event):
        try:
            self.power_control(1, "start")
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
        print "Enable CB1"
    def EnableCB2(self, event):
        try:
            self.power_control(2, "start")
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
        print "Enable CB2"

    def StandbyCB0(self, event):
        try:
            self.power_control(0, "stop")
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
        print "Standby CB0"
    def StandbyCB1(self, event):
        try:
            self.power_control(1, "stop")
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
        print "Standby CB1"
    def StandbyCB2(self, event):
        try:
            self.power_control(2, "stop")
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
        print "Standby CB2"

    def DisableCB0(self, event):
        try:
            self.power_control(0, "reset")
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
        print "Disable CB0"
    def DisableCB1(self, event):
        try:
            self.power_control(1, "reset")
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
        print "Disable CB1"
    def DisableCB2(self, event):
        try:
            self.power_control(2, "reset")
        except rospy.ServiceException, e:
            print "Service Call Failed: %s"%e
        print "Disable CB2"
