#! /usr/bin/python

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

import roslib
roslib.load_manifest('executive_python')
import rospy
import random
from std_msgs.msg import *
from robot_msgs.msg import *
from std_msgs.msg import *
from robot_msgs.msg import *
from battery_monitor_adapter import *
from navigation_adapter import *

class Executive:
  def __init__(self, goals, chrg_stations, navigator, batt_monitor, cycle_time):
    rospy.init_node("Executive", anonymous=True)
    self.goals = goals
    self.chrg_stations = chrg_stations
    self.navigator = navigator
    self.batt_monitor = batt_monitor
    self.cycle_time = cycle_time
    self.state = "nav"
    self.current_goal = self.goals[0]
    self.plugged_email_sent = False
    self.unplug_email_sent = False

  def legalStates(self):
    return self.navigator.legalState() and self.batt_monitor.legalState()

  def doCycle(self):
    #make sure that all adapters have legal states
    if self.legalStates():
      if self.state == "nav":
        """ 
          State Transitions from nav:
            1) nav --- robot is plugged in ---> recharge
            2) nav --- battery level is low --> nav_charge
            3) nav --- robot is inactive initially, reaches a goal, or timeout is reached ---> nav
            4) nav --- robot is inactive and should be pursuing the current goal (move_base crashed) ---> nav
        """
        if self.batt_monitor.pluggedIn():
          self.state = "recharge"
          print "nav --> recharge"
        elif self.batt_monitor.chargeNeeded():
          chrg_pts = self.chrg_stations[random.randint(0, len(self.chrg_stations) - 1)]
          self.navigator.sendGoal(chrg_pts, "map")
          self.state = "nav_charge"
          print "nav --> nav_charge"
        elif self.navigator.goalReached() or self.navigator.aborted() or (not self.navigator.active() and self.current_goal == None) or self.navigator.timeUp():
          self.current_goal = self.goals[random.randint(0, len(self.goals) - 1)]
          self.navigator.sendGoal(self.current_goal, "map")
          print "nav --> nav"
        elif not self.navigator.active() and self.current_goal != None:
          self.navigator.sendGoal(self.current_goal, "map")
          print "nav --> nav"
      elif self.state == "recharge":
        """
          State Transitions from recharge:
            1) recharge --- robot is done charging ---> nav
        """
        if (not self.plugged_email_sent) and self.batt_monitor.pluggedIn():
          self.batt_monitor.sendPluggedEmail()
          self.plugged_email_sent = True
        if self.batt_monitor.chargeDone():
          if(self.batt_monitor.pluggedIn()):
            if not self.unplug_email_sent:
              self.batt_monitor.sendUnPlugEmail()
              self.unplug_email_sent = True
          else:
            self.unplug_email_sent = False
            self.plugged_email_sent = False
            self.batt_monitor.sendUnpluggedEmail()
            #resume the current goal
            self.navigator.sendGoal(self.current_goal, "map")
            self.state = "nav"
            print "recharge --> nav"
      elif self.state == "nav_charge":
        """
          State Transitions from nav_charge
          1) nav_charge --- robot has reached charging station ---> recharge
          2) nav_charge --- robot is inactive or timeout is reached --> nav_charge
        """
        if self.navigator.goalReached():
          self.batt_monitor.sendPlugEmail()
          self.state = "recharge"
          print "nav_charge --> recharge"
        elif not self.navigator.active() or self.navigator.timeUp():
          chrg_pts = self.chrg_stations[random.randint(0, len(self.chrg_stations) - 1)]
          self.navigator.sendGoal(chrg_pts, "map")
          print "nav_charge --> nav_charge"
    else:
      if not self.navigator.legalState():
        print("Waiting on %s to be published" % (self.navigator.state_topic))
        rospy.logout("Waiting on %s to be published" % (self.navigator.state_topic))

      if not self.batt_monitor.legalState():
        print("Waiting on %s to be published" % (self.batt_monitor.state_topic))
        rospy.logout("Waiting on %s to be published" % (self.batt_monitor.state_topic))

  def run(self):
    while not rospy.is_shutdown():
      start = rospy.get_time()
      self.doCycle()
      end = rospy.get_time()
      sleep_time = self.cycle_time - (end - start)
      if sleep_time > 0:
        rospy.sleep(sleep_time)
      else:
        print("Executive missed cycle time of %.2f seconds" % (self.cycle_time))
        rospy.logwarn("Executive missed cycle time of %.2f seconds" % (self.cycle_time))

if __name__ == '__main__':
  try:
    batt_monitor = BatteryMonitorAdapter(.25, .75, "battery_state", ["watts@willowgarage.com", "eitan@willowgarage.com", "pr2-users@lists.willowgarage.com"], "pre", "/usr/sbin/sendmail")
    navigator = NavigationAdapter(30, 300, "/move_base/feedback", "/move_base/activate")

    goals = [
     [[18.912, 28.568, 0.000], [0.000, 0.000, 0.000, 1.000]],
     [[12.118, 39.812, 0.000], [0.000, 0.000, 0.000, 1.000]],
     [[47.350, 40.655, 0.000], [0.000, 0.000, -0.707, 0.707]],
     [[54.162, 20.087, 0.000], [0.000, 0.000, -0.160, 0.987]],
     [[50.659, 6.916, 0.000], [0.000, 0.000, 0.924, -0.383]],
     [[18.542, 12.301, 0.000], [0.000, 0.000, 0.906, -0.424]]
     ]

    chrg_stations = [
    [[18.098, 21.015, 0.000], [0.000, 0.000, 0.713, 0.701]]
    ]

    executive = Executive(goals, chrg_stations, navigator, batt_monitor, 1.0)
    executive.run()
  except KeyboardInterrupt, e:
    pass
  print "exiting"


