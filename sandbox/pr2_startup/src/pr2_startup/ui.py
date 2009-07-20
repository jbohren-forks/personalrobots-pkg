#!/usr/bin/env python
#
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

PKG = 'pr2_startup'

import roslib
import roslib.packages
roslib.load_manifest(PKG)

import sys
import rospy
import roslaunch
import roslaunch.pmon
from socket import gethostname
import threading, time
import traceback

import wx
import time
from wx import xrc


import wg_hardware_roslaunch.roslaunch_caller as roslaunch_caller

class PR2StartUpAppFrame(wx.Frame):
  def __init__(self, parent):
    wx.Frame.__init__(self, parent, wx.ID_ANY, "PR2StartUp")
    self.Bind(wx.EVT_CLOSE, self.OnExit)
    
    self._robots = { 'PRE' : 'pre.launch', 
                     'PRF' : 'prf.launch', 
                     'PRG' : 'prg.launch'}
    
    
    self._core_launcher = None
    self._robot_launcher = None
    self._joystick_launcher = None
    self._dashboard_launcher = None
    self._rviz_launcher = None
    self._rxgraph_launcher = None
    self._robot_launch = None

    self._mutex = threading.Lock()
    
    xrc_path = roslib.packages.get_pkg_dir(PKG) + '/xrc/pr2_start_up_panel.xrc'
    
    self._xrc = xrc.XmlResource(xrc_path)
    self._real_panel = self._xrc.LoadPanel(self, 'PR2StartUpPanel')
    

    self._real_panel.Bind(wx.EVT_BUTTON, self.LaunchCore, id=xrc.XRCID('m_button7'))
    self._real_panel.Bind(wx.EVT_BUTTON, self.LaunchRobot, id=xrc.XRCID('m_button8'))
    self._real_panel.Bind(wx.EVT_BUTTON, self.LaunchJoystick, id=xrc.XRCID('m_button9'))
    self._real_panel.Bind(wx.EVT_BUTTON, self.LaunchDashboard, id=xrc.XRCID('m_button10'))
    self._real_panel.Bind(wx.EVT_BUTTON, self.LaunchRviz, id=xrc.XRCID('m_button11'))
    self._real_panel.Bind(wx.EVT_BUTTON, self.LaunchRxgraph, id=xrc.XRCID('m_button12'))
    self._real_panel.Bind(wx.EVT_CHOICE, self.ChooseRobot, id=xrc.XRCID('m_choice1'))

    self.core_status = xrc.XRCCTRL(self._real_panel, 'm_textCtrl4')
    self.robot_status = xrc.XRCCTRL(self._real_panel, 'm_textCtrl5')
    self.joystick_status = xrc.XRCCTRL(self._real_panel, 'm_textCtrl6')
    self.robot_choice = xrc.XRCCTRL(self._real_panel, 'm_choice1')
    self.robot_choice.SetItems(self._robots)

    self.core_status.SetEditable(False)
    self.robot_status.SetEditable(False)
    self.joystick_status.SetEditable(False)
    self.core_status.SetBackgroundColour("Red")
    self.robot_status.SetBackgroundColour("Red")
    self.joystick_status.SetBackgroundColour("Red")

  def LaunchCore(self, event):
    print "launching core"
    if self._core_launcher == None:
      self._core_launcher = roslaunch_caller.launch_core()
      self.core_status.SetBackgroundColour("Light Green")
    else:
      self._core_launcher.stop()
      self._core_launcher =None
      self.core_status.SetBackgroundColour("Red")

  def LaunchRobot(self, event):
    print "launching robot" 
    if self._robot_launcher == None:
      if self._robot_launch == None:
        self._help_popup = wx.MessageDialog(self._real_panel, 'Please select a robot', 'caption', wx.OK)  
        self._help_popup.ShowModal()
      else:
        script = roslib.packages.get_pkg_dir('pr2_alpha') +'/'+ self._robot_launch
        self._robot_launcher = self.launch_script(script, None)
        self._robot_launcher.spin_once()
        self.robot_status.SetBackgroundColour("Light Green")
    else:
      self._robot_launcher.shutdown()
      self._robot_launcher = None
      self.robot_status.SetBackgroundColour("Red")

  def LaunchJoystick(self, event):
    print "launching joystick"
    if self._joystick_launcher == None:
      script = roslib.packages.get_pkg_dir('pr2_alpha') +'/'+ 'teleop_joystick.launch'
      self._joystick_launcher = self.launch_script(script, None)
      self._joystick_launcher.spin_once()
      self.joystick_status.SetBackgroundColour("Light Green")
    else:
      self._joystick_launcher.shutdown()
      self._joystick_launcher =None
      self.joystick_status.SetBackgroundColour("Red")
  
  def LaunchDashboard(self, event):
    print "launching dashboard"
    if self._dashboard_launcher == None:
      script = roslib.packages.get_pkg_dir(PKG) + '/launch/dashboard.launch'
      self._dashboard_launcher = self.launch_script(script, None)
      self._dashboard_launcher.spin_once()
    else:
      self._dashboard_launcher.shutdown()
      self._dashboard_launcher = None
  
  def LaunchRviz(self, event):
    print "launching dashboard"
    if self._rviz_launcher == None:
      script = roslib.packages.get_pkg_dir(PKG) + '/launch/rviz.launch'
      self._rviz_launcher = self.launch_script(script, None)
      self._rviz_launcher.spin_once()
    else:
      self._rviz_launcher.shutdown()
      self._rviz_launcher = None
  
  def LaunchRxgraph(self, event):
    print "launching dashboard"
    if self._rxgraph_launcher == None:
      script = roslib.packages.get_pkg_dir(PKG) + '/launch/rxgraph.launch'
      self._rxgraph_launcher = self.launch_script(script, None)
      self._rxgraph_launcher.spin_once()
    else:
      self._rxgraph_launcher.shutdown()
      self._rxgraph_launcher = None

  def ChooseRobot(self, event):
    self._robot_launch = self._robots[self.robot_choice.GetStringSelection()]

  def launch_script(self, script, process_listener_object = None):  
    f = open(script, 'r')
    launch_xml = f.read()
    f.close()

    launch = roslaunch_caller.ScriptRoslaunch(launch_xml, process_listener_object)
    try:
      launch.start()
    except roslaunch.RLException, e:
      traceback.print_exc()
      return None
  
    return launch

  def OnExit(self,e):
    print "existing frame"
    if self._core_launcher != None:   
      self._core_launcher.stop()
    self.Destroy()
    wx.GetApp().ProcessIdle()
    

class PR2StartUpApp(wx.App):
  def OnInit(self):
    
    self._frame = PR2StartUpAppFrame(None)
    self._frame.SetSize(wx.Size(300,350))
    self._frame.Layout()
    self._frame.Centre()
    self._frame.Show(True)
    
    return True

    
if __name__ == '__main__':
  try:
    app = PR2StartUpApp(0)
    app.MainLoop()
  except Exception, e:
    print e
    traceback.print_exc()
    
  print 'Quitting PR2 start up app'
  sys.exit(0)


