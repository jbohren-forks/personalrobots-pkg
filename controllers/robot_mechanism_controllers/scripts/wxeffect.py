#!/usr/bin/env python

PKG = 'robot_mechanism_controllers'
import roslib; roslib.load_manifest(PKG)

import rospy
from robot_msgs.msg import *

import wx
from wx import xrc

import random
from std_msgs.msg import *
from robot_srvs.srv import SpawnController, KillController


class wxeffect_panel(wx.Panel):
  def __init__(self, parent):
    wx.Panel.__init__(self, parent, wx.ID_ANY)

    # Create interface objects
    box = wx.StaticBox(self, -1, "Effect")
    bsizer = wx.StaticBoxSizer(box, wx.VERTICAL)
    self.SetSizer(bsizer)
    joint_label = wx.StaticText(self, label="Joint:")
    self.joint_choice = wx.Choice(self, -1, (100, 50), choices = ['<none>'])
    effort_label = wx.StaticText(self, label="Effort:")
    self.effort_slider = wx.Slider(self, 100, 0, -100, 100, (30, 60), (250, -1), wx.SL_HORIZONTAL|wx.SL_AUTOTICKS)
    self.effort_slider.Enable(0)

    # Layout objects
    grid = wx.FlexGridSizer(cols=2, hgap=4, vgap=4)

    grid.Add(joint_label, flag=wx.ALIGN_CENTER_VERTICAL)
    grid.Add(self.joint_choice, flag=wx.ALIGN_CENTER_VERTICAL)

    grid.Add(effort_label, flag=wx.ALIGN_CENTER_VERTICAL)
    grid.Add(self.effort_slider, flag=wx.ALIGN_CENTER_VERTICAL)
    bsizer.Add(grid)

    # Bind events
    self.joint_choice.Bind(wx.EVT_CHOICE, self.choice_callback)
    self.effort_slider.Bind(wx.EVT_SCROLL, self.effort_callback)

    # register for ROS topics and services
    rospy.wait_for_service('spawn_controller')
    self.spawn = rospy.ServiceProxy('spawn_controller', SpawnController)
    self.kill = rospy.ServiceProxy('kill_controller', KillController)
    self.subscription = rospy.Subscriber("/mechanism_state", MechanismState, self.mechanism_state_callback)

    self.command_pub = None
    self.current = None

  def xml_for(self, joint):
    self.current = "quick_effort_controller_%08d" % random.randint(0, 10**8-1)
    return "\
  <controller name=\"%s\" type=\"JointEffortControllerNode\">\
  <joint name=\"%s\" />\
  </controller>" % (self.current, joint)

  def cleanup_controller(self):
    if self.current is not None:
      self.kill(self.current)
      self.current = None
    if self.command_pub is not None:
      self.command_pub.unregister()
      self.command_pub = None

  def choice_callback(self, event):
    # Cleanup state from previous controller, if any
    self.cleanup_controller()

    # Spawn new controller and create set_command publisher
    self.effort_slider.SetValue(0)
    joint = str(event.GetString())
    if joint == '<none>':
      self.effort_slider.Enable(0)
    else:
      resp = self.spawn(self.xml_for(joint))
      if len(resp.ok) < 1 or not resp.ok[0]:
        print "Failed to spawn effort controller for %s" % joint
        sys.exit(1)
      self.effort_slider.Enable(1)
      self.command_pub = rospy.Publisher("/%s/set_command" % self.current, Float64)

  def mechanism_state_callback(self, message):
    self.subscription.unregister()
    wx.CallAfter(self.add_joints, message)

  def add_joints(self, message):
    for j in message.joint_states:
      self.joint_choice.Append(j.name)
    self.joint_choice.InvalidateBestSize()
    self.joint_choice.SetSize(self.joint_choice.GetBestSize())
    self.Layout()

  def effort_callback(self, event):
    effort = float(event.GetPosition())/1000.
    print effort
    if self.command_pub is not None:
      self.command_pub.publish(Float64(effort))

  def quit(self):
    self.cleanup_controller()

class MainWindow(wx.Frame):
  def __init__(self, parent, id, title):
    wx.Frame.__init__(self, parent, wx.ID_ANY, title)
    self.filemenu = wx.Menu()
    self.filemenu.Append(wx.ID_EXIT, "E&xit"," Exit the program")
    self.menubar = wx.MenuBar()
    self.menubar.Append(self.filemenu,"&File")
    self.SetMenuBar(self.menubar)
    wx.EVT_MENU(self, wx.ID_EXIT, self.on_exit)
    self.Bind(wx.EVT_CLOSE, self.on_close)

    self.panel = wxeffect_panel(self)

    self.SetSize(wx.Size(350,140))

  def on_exit(self, e):
    self.Close(True)

  def on_close(self, event):
    self.panel.quit()
    self.Destroy()

def listener():
  app = wx.PySimpleApp()
  rospy.init_node('wxeffect', anonymous=True)

  frame = MainWindow(None, -1, "wxeffect")
  frame.Show()

  app.MainLoop()

if __name__ == '__main__':
  try:
    listener()
  except KeyboardInterrupt, e:
    pass
