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

import roslib
roslib.load_manifest('pr2_dashboard2')

import wx
import wx.aui
import wx.py.shell
import rxtools

# TODO: remove once 0.7.1 is released
try:
    import rxtools.cppwidgets as rxtools
except ImportError:
    pass

import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel
from pr2_msgs.msg import PowerState, PowerBoardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest
import std_srvs.srv

import rospy

from os import path
import threading

class DiagnosticsFrame(wx.Frame):
  def __init__(self, parent, id, title):
    wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition, wx.Size(600, 200))
    
    self.Bind(wx.EVT_CLOSE, self.on_close)
    
    self._diagnostics_panel = RobotMonitorPanel(self)
    
  def on_close(self, evt):
    self.Hide()
    
class RosoutFrame(wx.Frame):
  def __init__(self, parent, id, title):
    wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition, wx.Size(800, 200))
    
    self.Bind(wx.EVT_CLOSE, self.on_close)
    
    self._rosout_panel = rxtools.RosoutPanel(self)
    
  def get_panel(self):
    return self._rosout_panel
    
  def on_close(self, evt):
    self.Hide()
    
class PowerStateControl(wx.Window):
  def __init__(self, parent, id, icons_path):
    wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(60, -1))
    
    self._power_consumption = 0.0
    self._pct = 0
    self._time_remaining = 0
    self._ac_present = 0
    
    self._left_bitmap = wx.Bitmap(path.join(icons_path, "battery_left.png"), wx.BITMAP_TYPE_PNG)
    self._right_bitmap = wx.Bitmap(path.join(icons_path, "battery_right.png"), wx.BITMAP_TYPE_PNG)
    self._plug_bitmap = wx.Bitmap(path.join(icons_path, "plug.png"), wx.BITMAP_TYPE_PNG)
    self._start_x = self._left_bitmap.GetWidth()
    self._end_x = self.GetSize().x - self._right_bitmap.GetWidth()
    self._width = self._end_x - self._start_x
    
    self._plugged_in = False
    
    self.Bind(wx.EVT_PAINT, self.on_paint)

  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)

    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()
 
    w = self.GetSize().GetWidth()
    h = self.GetSize().GetHeight()   
    red = [255, 0.0, 0.0]
    yellow = [255, 255, 0.0]
    green = [0.0, 255, 0.0]
    
    color = None
    if (self._pct > 0.5):
      color = wx.Colour(green[0], green[1], green[2])
    elif (self._pct > 0.3):
      color = wx.Colour(yellow[0], yellow[1], yellow[2])
    else:
      color = wx.Colour(red[0], red[1], red[2])
    
    dc.SetBrush(wx.GREY_BRUSH)
    dc.SetPen(wx.GREY_PEN)
    dc.DrawRectangle(self._start_x, 0, self._width, h)
    
    dc.SetBrush(wx.Brush(color))
    dc.SetPen(wx.Pen(color))
    dc.DrawRectangle(self._start_x, 0, self._width * (self._pct), h)
    dc.DrawBitmap(self._left_bitmap, 0, 0, True)
    dc.DrawBitmap(self._right_bitmap, self._end_x, 0, True)
    
    if (self._plugged_in):
      dc.DrawBitmap(self._plug_bitmap, 
                    (self._start_x + self._width) / 2.0 - (self._plug_bitmap.GetWidth() / 2.0), 
                    self.GetSize().GetHeight() / 2.0 - (self._plug_bitmap.GetHeight() / 2.0))
      
  def set_power_state(self, msg):
    self._power_consumption = msg.power_consumption
    self._time_remaining = msg.time_remaining
    self._pct = msg.relative_capacity / 100.0
    self._plugged_in = msg.AC_present
    
    self.SetToolTip(wx.ToolTip("Battery: %.2f%% (%d minutes remaining)"%(self._pct * 100, self._time_remaining)))
    
    self.Refresh()
    
class StatusControl(wx.Window):
  def __init__(self, parent, id, bitmap, icons_path, lights = ("green_light.png", "yellow_light.png", "red_light.png", "grey_light.png")):
    wx.Window.__init__(self, parent, id)
    self.SetSize(wx.Size(32, 32))
    
    self._bitmap = bitmap
    
    self._green = wx.Bitmap(path.join(icons_path, lights[0]), wx.BITMAP_TYPE_PNG)
    self._yellow = wx.Bitmap(path.join(icons_path, lights[1]), wx.BITMAP_TYPE_PNG)
    self._red = wx.Bitmap(path.join(icons_path, lights[2]), wx.BITMAP_TYPE_PNG)
    self._stale = wx.Bitmap(path.join(icons_path, lights[3]), wx.BITMAP_TYPE_PNG)
    
    self.set_stale()
    
    self.Bind(wx.EVT_PAINT, self.on_paint)
    self.Bind(wx.EVT_LEFT_UP, self.on_left_up)

  def on_left_up(self, evt):
    x = evt.GetX()
    y = evt.GetY()
    if (x >= 0 and y >= 0 and x < self.GetSize().GetWidth() and y < self.GetSize().GetHeight()):
      event = wx.CommandEvent(wx.EVT_BUTTON._getEvtType(), self.GetId())
      wx.PostEvent(self, event)

  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)
    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()
    
    size = self.GetSize();
    
    dc.DrawBitmap(self._color, (size.GetWidth() - self._color.GetWidth()) / 2.0, (size.GetHeight() - self._color.GetHeight()) / 2.0, True)
    dc.DrawBitmap(self._bitmap, (size.GetWidth() - self._bitmap.GetWidth()) / 2.0, (size.GetHeight() - self._bitmap.GetHeight()) / 2.0, True)

  def set_ok(self):
    self._color = self._green
    self.Refresh()
    
  def set_warn(self):
    self._color = self._yellow
    self.Refresh()
    
  def set_error(self):
    self._color = self._red
    self.Refresh()
    
  def set_stale(self):
    self._color = self._stale
    self.Refresh()
    
class BreakerControl(StatusControl):
  def __init__(self, parent, id, icons_path):
    StatusControl.__init__(self, parent, id, wx.Bitmap(path.join(icons_path, "electricity.png"), wx.BITMAP_TYPE_PNG), icons_path)
    
    self.Bind(wx.EVT_PAINT, self.on_paint)
    self.Bind(wx.EVT_LEFT_DOWN, self.on_left_down)
    
    self._power_control = rospy.ServiceProxy('power_board_control', PowerBoardCommand)
    self._serial = 0
    try:
      param = rospy.search_param("power_board_serial")
      
      if (param and len(param) > 0):
        self._serial = int(rospy.get_param(param))
    except rospy.ROSException, e:
      pass
    except KeyError, e:
      pass
    
  def on_left_down(self, evt):
    menu = wx.Menu()
    menu.Bind(wx.EVT_MENU, self.on_enable_all, menu.Append(wx.ID_ANY, "Enable All"))
    menu.Bind(wx.EVT_MENU, self.on_standby_all, menu.Append(wx.ID_ANY, "Standby All"))
    menu.Bind(wx.EVT_MENU, self.on_disable_all, menu.Append(wx.ID_ANY, "Disable All"))
    
    self.PopupMenu(menu)
    
  def control(self, breaker, cmd):
    try:
      power_cmd = PowerBoardCommandRequest()
      power_cmd.breaker_number = breaker
      power_cmd.command = cmd
      power_cmd.serial_number = self._serial
      self._power_control(power_cmd)
      
      return True
    except rospy.ServiceException, e:
      wx.MessageBox("Service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
      
    return False
  
  def control3(self, cmd):
    if (not self.control(0, cmd)):
      return False
    if (not self.control(1, cmd)):
      return False
    if (not self.control(2, cmd)):
      return False
    
    return True
    
  def on_enable_all(self, evt):
    if (not self.control3("reset")):
      return
    
    self.control3("start")
  
  def on_standby_all(self, evt):
    if (not self.control3("reset")):
      return
    
    self.control3("stop")
  
  def on_disable_all(self, evt):
    self.control3("disable")
    
  def set_power_board_state_msg(self, msg):
    self._power_board_state = msg
    
    status_msg = "OK"
    
    if (msg.circuit_state[0] == PowerBoardState.STATE_DISABLED or
        msg.circuit_state[1] == PowerBoardState.STATE_DISABLED or
        msg.circuit_state[2] == PowerBoardState.STATE_DISABLED):
      self.set_error()
      status_msg = "Disabled"
    elif (msg.circuit_state[0] == PowerBoardState.STATE_NOPOWER or
          msg.circuit_state[1] == PowerBoardState.STATE_NOPOWER or
          msg.circuit_state[2] == PowerBoardState.STATE_NOPOWER):
      self.set_error()
      status_msg = "No Power"
    elif (msg.circuit_state[0] == PowerBoardState.STATE_STANDBY or
          msg.circuit_state[1] == PowerBoardState.STATE_STANDBY or
          msg.circuit_state[2] == PowerBoardState.STATE_STANDBY):
      self.set_warn()
      status_msg = "Standby"
    else:
      self.set_ok()
      
    self.SetToolTip(wx.ToolTip("Breaker State [%s]"%(status_msg)))  

class PR2Frame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def __init__(self, parent, id=wx.ID_ANY, title='PR2 Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        wx.InitAllImageHandlers()
        
        rospy.init_node('pr2_dashboard2', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("pr2_dashboard2_cpp", anonymous=True)
        except AttributeError:
            pass
        
        icons_path = path.join(roslib.packages.get_pkg_dir('pr2_dashboard2'), "icons/")
        self._diagnostics_bitmap = wx.Bitmap(path.join(icons_path, "diagnostics.png"), wx.BITMAP_TYPE_PNG)
        self._rosout_bitmap = wx.Bitmap(path.join(icons_path, "speech_bubble.png"), wx.BITMAP_TYPE_PNG)
        
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)
        
        # Diagnostics
        self._diagnostics_button = StatusControl(self, wx.ID_ANY, self._diagnostics_bitmap, icons_path)
        self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics"))
        sizer.Add(self._diagnostics_button, 0)
        
        # Rosout
        self._rosout_button = StatusControl(self, wx.ID_ANY, self._rosout_bitmap, icons_path)
        self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
        sizer.Add(self._rosout_button, 0)
        
        # Separator
        #sizer.Add(wx.StaticLine(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_VERTICAL), 0, wx.EXPAND)
        
        # Breakers
        self._breakers_ctrl = BreakerControl(self, wx.ID_ANY, icons_path)
        self._breakers_ctrl.SetToolTip(wx.ToolTip("Breakers"))
        sizer.Add(self._breakers_ctrl, 0)
        
        # Motors
        self._motors_button = StatusControl(self, wx.ID_ANY, wx.NullBitmap, icons_path)
        self._motors_button.SetToolTip(wx.ToolTip("Motors"))
        sizer.Add(self._motors_button, 0)
        self._motors_button.Bind(wx.EVT_LEFT_DOWN, self.on_motors_clicked)
        
        # Separator
        sizer.Add(wx.StaticLine(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_VERTICAL), 0, wx.EXPAND)
        
        # run-stop
        self._runstop_ctrl = StatusControl(self, wx.ID_ANY, wx.NullBitmap, icons_path, lights=("green_stoplight.png", "yellow_stoplight.png", "red_stoplight.png", "grey_stoplight.png"))
        self._runstop_ctrl.SetToolTip(wx.ToolTip("Runstop Status"))
        sizer.Add(self._runstop_ctrl, 0)
        
        # Wireless run-stop
        self._wireless_runstop_ctrl = StatusControl(self, wx.ID_ANY, wx.NullBitmap, icons_path, lights=("green_stoplight.png", "yellow_stoplight.png", "red_stoplight.png", "grey_stoplight.png"))
        self._wireless_runstop_ctrl.SetToolTip(wx.ToolTip("Wireless Runstop Status"))
        sizer.Add(self._wireless_runstop_ctrl, 0)
        
        # Separator
        sizer.Add(wx.StaticLine(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_VERTICAL), 0, wx.EXPAND)
        
        # Battery State
        self._power_state_ctrl = PowerStateControl(self, wx.ID_ANY, icons_path)
        self._power_state_ctrl.SetToolTip(wx.ToolTip("Battery"))
        sizer.Add(self._power_state_ctrl, 1, wx.EXPAND)
        
        self._config = wx.Config("pr2_dashboard")
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.Layout()
        self.Fit()
        
        self._diagnostics_frame = DiagnosticsFrame(self, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_button.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)
        
        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)
        
        self.load_config()
        
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(100)
        
        self._mutex = threading.Lock()
        rospy.Subscriber("power_state", PowerState, self.power_callback)
        rospy.Subscriber("power_board_state", PowerBoardState, self.power_board_callback)
        
    def on_timer(self, evt):
      level = self._diagnostics_frame._diagnostics_panel.get_top_level_state()
      if (level >= 2):
        self._diagnostics_button.set_error()
      elif (level == 1):
        self._diagnostics_button.set_warn()
      elif (level == -1):
        self._diagnostics_button.set_stale()
      else:
        self._diagnostics_button.set_ok()
        
      self.update_rosout()
        
      if (rospy.is_shutdown()):
        self.Close()
        
    def on_diagnostics_clicked(self, evt):
      self._diagnostics_frame.Show()
      self._diagnostics_frame.Raise()
      
    def on_rosout_clicked(self, evt):
      self._rosout_frame.Show()
      self._rosout_frame.Raise()
      
    def on_motors_clicked(self, evt):
      menu = wx.Menu()
      menu.Bind(wx.EVT_MENU, self.on_reset_motors, menu.Append(wx.ID_ANY, "Reset"))
      menu.Bind(wx.EVT_MENU, self.on_halt_motors, menu.Append(wx.ID_ANY, "Halt"))
      self.PopupMenu(menu)
      
    def on_reset_motors(self, evt):
      reset = rospy.ServiceProxy("reset_motors", std_srvs.srv.Empty)
       
      try:
        reset()
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to reset the motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
    
    def on_halt_motors(self, evt):
      halt = rospy.ServiceProxy("halt_motors", std_srvs.srv.Empty)
       
      try:
        halt()
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to halt the motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
      
    def power_callback(self, msg):
      self._mutex.acquire()
      self._power_message = msg
      self._mutex.release()
      
      wx.CallAfter(self.new_power_message) 
      
    def new_power_message(self):
      self._mutex.acquire()
      msg = self._power_message
      self._power_message = None
      self._mutex.release()
      
      if (msg is not None):
        self._power_state_ctrl.set_power_state(msg)
        
    def power_board_callback(self, msg):
      self._mutex.acquire()
      self._power_board_message = msg
      self._mutex.release()
      
      wx.CallAfter(self.new_power_board_message) 
      
    def new_power_board_message(self):
      self._mutex.acquire()
      msg = self._power_board_message
      self._power_board_message = None
      self._mutex.release()
      
      if (msg is not None):
        self._breakers_ctrl.set_power_board_state_msg(msg)
        if (not msg.run_stop):
          # if the wireless stop is also off, we can't tell if the runstop is pressed or not
          if (not msg.wireless_stop):
            self._runstop_ctrl.set_warn()
          else:
            self._runstop_ctrl.set_error()
        else:
          self._runstop_ctrl.set_ok()
          
        if (not msg.wireless_stop):
          self._wireless_runstop_ctrl.set_error()
        else:
          self._wireless_runstop_ctrl.set_ok()
          
    def update_rosout(self):
      # Support pre-ROS-0.8, just don't have this functionality
      try:
          getattr(self._rosout_frame.get_panel(), "getMessageSummary")
      except AttributeError:
          return
      
      summary = self._rosout_frame.get_panel().getMessageSummary(30.0)
      
      if (summary.fatal or summary.error):
        self._rosout_button.set_error()
      elif (summary.warn):
        self._rosout_button.set_warn()
      else:
        self._rosout_button.set_ok()
        
        
      tooltip = ""
      if (summary.fatal):
        tooltip += "\nFatal: %s"%(summary.fatal)
      if (summary.error):
        tooltip += "\nError: %s"%(summary.error)
      if (summary.warn):
        tooltip += "\nWarn: %s"%(summary.warn)
      if (summary.info):
        tooltip += "\nInfo: %s"%(summary.info)
      if (summary.debug):
        tooltip += "\nDebug: %s"%(summary.debug)
      
      if (len(tooltip) == 0):
        tooltip = "Rosout: no recent activity"
      else:
        tooltip = "Rosout: recent activity:" + tooltip
        
      self._rosout_button.SetToolTip(wx.ToolTip(tooltip))
        
    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)
      
      self.SetPosition((x, y))
      self.SetSize((width, height))
        
    def save_config(self):
      config = self._config
      
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)
      
      config.Flush()
        
    def on_close(self, event):
      self.save_config()
      
      self.Destroy()
      
