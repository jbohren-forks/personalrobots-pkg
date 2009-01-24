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

# Author: Brian Gerkey (modelled after Eitan's executive.py)

# States:
#
# IDLE - this is the start state
#
# SLOWSCAN - not moving, slow-scanning
#
# MOVEBASE - moving base to goal, slow-scanning
# 
# MOVETOGRASP - moving EE to goal near object, fast-scanning
#
# GRASPING - closing gripper
#
# RAISING - moving EE to goal above table, no EE collision detection
#
# STOWING - moving EE to pre-defined stow configuration, fast-scanning, 
#           object joined to robot, subtracted from map
#
# MOVETODROP - moving EE to goal near landing zone, fast-scanning, object
#              joined to robot, subtracted from map (same as STOWING, with
#              different goal?)
#
# DROPPING - opening gripper
#

# Fault-free state sequence:
#
# 1. IDLE
#
# 2. SLOWSCAN (look for table / objects)
#  Collision map returns table geometry and list of object geometries.
#  Pick an object
#  Compute a goal base pose near the table & object
#
# 3. MOVEBASE (go to table)
#  Send goal base pose to MoveBase
#
# 4. SLOWSCAN (look for table / objects)
#  Collision map returns table geometry and list of object geometries.
#  Pick an object (pick the same one somehow?)
#
# 5. MOVETOGRASP
#  Send object pose as constraint-based EE goal to MoveArm
#
# 6. GRASPING
#  Blindly close, grasping object
#
# 7. RAISING
#  Disable EE collision body
#  Compute EE goal fixed distance in +Z from table surface (remember table
#    geometry from step 4; could SLOWSCAN again if necessary)
#  Give EE goal to MoveArm
#  Enable EE collision body
#  Remove object geom from collision map
#  Add object geom to robot collision model (AttachedObject message)
#
# 8. STOWING
#  Send pre-defined stow configuration to MoveArm
#
# 9. SLOWSCAN (look for table / landing zone)
#  Collision map returns table geometry and convex hulls for empty
#    regions
#  Pick empty region
#  Compute point in region
#  Compute goal base pose near point & table
#
#10. MOVEBASE (go to landing zone)
#  Send goal base pose to MoveBase
#
#11. SLOWSCAN
#  Validate chosen landing zone
#  Compute EE goal pose 
#  
#12. MOVETODROP
#  Send EE goal pose to MoveArm
#
#13. DROPPING
#  Blindly open gripper, releasing object
#  Remove object from body (empty AttachedObject list)
#
#14. STOWING
#  Send pre-defined stow configuration to MoveArm
#
#15. Goto (1)

import rostools
rostools.update_path('executive_python')
import rospy
import random
from std_msgs.msg import *
from robot_msgs.msg import *
from highlevel_controllers.msg import *
from navigation_adapter import *
#from movearm_adapter import *
#from tiltlaser_adapter import *
#from gripper_adapter import *

class Executive:
  def __init__(self, goals, navigator, cycle_time):
    rospy.init_node("Executive", anonymous=True)
    self.goals = goals
    self.navigator = navigator
    self.cycle_time = cycle_time
    self.state = "idle"
    self.current_goal = None

  def legalStates(self):
    return self.navigator.legalState()

  def doCycle(self):
    #make sure that all adapters have legal states
    if self.legalStates():
      if self.state == "idle":
        if self.navigator.goalReached() or (not self.navigator.active() and self.current_goal == None) or self.navigator.timeUp():
          self.current_goal = self.goals[random.randint(0, len(self.goals) - 1)]
          self.navigator.sendGoal(self.current_goal, "odom")
          print "nav --> nav"
        elif not self.navigator.active() and self.current_goal != None:
          self.navigator.sendGoal(self.current_goal, "odom")
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
        print("Executive missed cycle time of %.2f seconds" % (self.cycle_time))
        rospy.logwarn("Executive missed cycle time of %.2f seconds" % (self.cycle_time))

if __name__ == '__main__':
  try:
    navigator = NavigationAdapter(30, 300, "state", "goal")

    goals = [
     [50.250, 6.863, 3.083], 
     [18.550, 11.762, 2.554],
     [53.550, 20.163, 0.00],
     [18.850, 28.862, 0.00],
     [47.250, 39.162, 1.571],
     [11.450, 39.662, 0.00]
     ]

    chrg_stations = [
     [33.844, 36.379, -1.571]
    ]

    executive = Executive(goals, navigator, 1.0)
    executive.run()
  except KeyboardInterrupt, e:
    pass
  print "exiting"


