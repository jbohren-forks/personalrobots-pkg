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
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
import roslib
roslib.load_manifest('executive_python')
import rospy
import random
from std_msgs.msg import *
from robot_msgs.msg import *
from battery_monitor_adapter import *
from recharge_adapter import *
from navigation_adapter import *

class IndefiniteNav:
  def __init__(self, goals, navigator, cycle_time):
    rospy.init_node("IndefiniteNav", anonymous=True)
    self.goals = goals
    self.navigator = navigator
    self.cycle_time = cycle_time
    self.state = "nav"
    self.current_goal = self.goals[0]

  def legalStates(self):
    return self.navigator.legalState()

  def doCycle(self):
    #make sure that all adapters have legal states
    if self.legalStates():
      if self.state == "nav":
        """ 
          State Transitions from nav:
            1) nav --- robot is inactive initially, reaches a goal, or timeout is reached ---> nav
            2) nav --- robot is inactive and should be pursuing the current goal (move_base crashed) ---> nav
        """
        if self.navigator.goalReached() or (not self.navigator.active() and self.current_goal == None) or self.navigator.timeUp():
          self.current_goal = self.goals[random.randint(0, len(self.goals) - 1)]
          self.navigator.sendGoal(self.current_goal, "map")
          print "nav --> nav"
        elif not self.navigator.active() and self.current_goal != None:
          self.navigator.sendGoal(self.current_goal, "map")
          print "nav --> nav"
    else:
      if not self.navigator.legalState():
        print("Waiting on %s to be published" % (self.navigator.state_topic))
        rospy.logout("Waiting on %s to be published" % (self.navigator.state_topic))

  def run(self):
    while not rospy.is_shutdown():
      start = rospy.get_time()
      self.doCycle()
      end = rospy.get_time()
      sleep_time = self.cycle_time - (end - start)
      if sleep_time > 0:
        rospy.sleep(sleep_time)
      else:
        print("IndefiniteNav missed cycle time of %.2f seconds" % (self.cycle_time))
        rospy.logwarn("IndefiniteNav missed cycle time of %.2f seconds" % (self.cycle_time))

if __name__ == '__main__':
  try:
    #navigator = NavigationAdapter(30, 300, "/move_base_node/feedback", "/move_base_node/activate")
    navigator = NavigationAdapter(30, 300, "/move_base_node/feedback", "/goal")

    goals = [
     [[18.912, 28.568, 0.000], [0.000, 0.000, 0.000, 1.000]],
     [[12.118, 39.812, 0.000], [0.000, 0.000, 0.000, 1.000]],
     [[47.350, 40.655, 0.000], [0.000, 0.000, -0.707, 0.707]],
     [[54.162, 20.087, 0.000], [0.000, 0.000, -0.160, 0.987]],
     [[50.659, 6.916, 0.000], [0.000, 0.000, 0.924, -0.383]],
     [[18.542, 12.301, 0.000], [0.000, 0.000, 0.906, -0.424]]
     ]


    indefinite_nav = IndefiniteNav(goals, navigator, 1.0)
    indefinite_nav.run()
  except KeyboardInterrupt, e:
    pass
  print "exiting"


