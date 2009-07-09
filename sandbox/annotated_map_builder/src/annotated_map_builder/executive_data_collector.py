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
roslib.load_manifest('annotated_map_builder')
import rospy
import sys
import random
from std_msgs.msg import *
from robot_msgs.msg import *
from std_msgs.msg import *
from robot_msgs.msg import *
from sensor_msgs.msg import RawStereo

import tf
from battery_monitor_adapter import *
from navigation_adapter import *
from stuck_adapter import *
from manual_charge_adapter import *

from annotated_map_builder.wait_for_k_messages_adapter import *
from annotated_map_builder.wait_for_multiple_head_configs import *
from annotated_map_builder.move_head_adapter import *
import annotated_map_builder.target_poses


class ExecutiveDataCollector:
  def __init__(self, goals, chrg_stations, navigator, batt_monitor, unstuck, manual_charger, capture_waiter, move_head_adapter, move_head_adapter2, cycle_time):
    self.goals = goals
    #self.capture_configs=capture_configs
    self.chrg_stations = chrg_stations
    self.navigator = navigator
    self.batt_monitor = batt_monitor
    self.capture_waiter = capture_waiter;
    self.move_head_adapter = move_head_adapter;
    self.move_head_adapter2 = move_head_adapter2;
    self.cycle_time = cycle_time

    self.state = "nav"
    self.current_goal_idx = 0
    self.current_goal = self.goals[self.current_goal_idx]
    self.num_goals_visited = 0;
    self.num_goals_failed = 0;
    self.plugged_email_sent = False
    self.unplug_email_sent = False
    self.unstuck = unstuck
    self.unstuck_time = rospy.get_time()
    self.stuck = False

  def legalStates(self):
    return self.navigator.legalState() and self.batt_monitor.legalState() and self.capture_waiter.legalState() and self.move_head_adapter.legalState() and self.move_head_adapter2.legalState()


  def selectNextGoal(self):
    if self.current_goal_idx<len(self.goals)-1:
      self.current_goal_idx += 1;
    else:
      self.current_goal_idx=0;
    self.current_goal = self.goals[self.current_goal_idx];

    print "Visited %d, Failed %d, Total %d" %(
      self.num_goals_visited, self.num_goals_failed ,len(self.goals))

  def doCycle(self):
    print "c",self.state,self.legalStates()
    #make sure that all adapters have legal states
    if self.legalStates():
      #check if we're stuck
      #if self.unstuck.stuck():
        #self.unstuck.getUnstuck()

      if self.unstuck.stuck() and self.unstuck_time + 15.0 < rospy.get_time() and not self.stuck:
        self.stuck = True
        print "Oh no, I'm stuck... asking for help"
        self.batt_monitor.sendStuckEmail()
      else:
        if self.stuck:
          print "I've gotten myself unstuck... calling off the help"
          self.batt_monitor.sendUnstuckEmail()
          self.stuck = False
        self.unstuck_time = rospy.get_time()

      if self.state == "nav":
        """ 
          State Transitions from nav:
            1) nav --- robot is plugged in ---> recharge
            2) nav --- battery level is low --> nav_charge
            3) nav --- reaches a goal --> capture
            4) nav --- robot is inactive initially,  or timeout is reached ---> nav
            5) nav --- robot is inactive and should be pursuing the current goal (move_base crashed) ---> nav
        """
        if self.batt_monitor.pluggedIn():
          self.state = "recharge"
          print "nav --> recharge"
        elif self.batt_monitor.chargeNeeded() or manual_charger.chargeRequested():
          chrg_pts = self.chrg_stations[random.randint(0, len(self.chrg_stations) - 1)]
          self.navigator.sendGoal(chrg_pts, "/map")
          self.state = "nav_charge"
          print "nav --> nav_charge"

        elif self.navigator.goalReached():
          self.state = "capture"
          self.capture_waiter.sendGoal();
          print "nav --> capture"
          self.move_head_adapter.sendPreempt(); 
          self.move_head_adapter2.sendGoal(); 

        elif self.navigator.aborted() or (not self.navigator.active() and self.current_goal == None) or self.navigator.timeUp():
          print "Aborted:",self.current_goal
          self.selectNextGoal();
          self.navigator.sendGoal(self.current_goal, "/map")
          self.move_head_adapter2.sendPreempt(); 
          self.move_head_adapter.sendGoal(); 

          print "nav --> nav"

        elif not self.navigator.active() and self.current_goal != None:
          self.navigator.sendGoal(self.current_goal, "/map")
          print "nav --> nav"
      elif self.state=="capture":
        print "C"
        if self.capture_waiter.timeUp() and not self.capture_waiter.success():
          print "ERROR: capture timed out"
        if self.capture_waiter.timeUp() or self.capture_waiter.success():
          #self.capture_waiter.sendDefaultHeadConfig()

          self.state="nav"
          self.move_head_adapter2.sendPreempt(); 
          self.move_head_adapter.sendGoal(); 


          self.selectNextGoal();
          self.navigator.sendGoal(self.current_goal, "/map")
          print "capture --> nav"
        #else:
        #  self.capture_waiter.doCycle()
          

      elif self.state == "recharge":
        """
          State Transitions from recharge:
            1) recharge --- robot is done charging ---> nav
        """
        if (not self.plugged_email_sent) and self.batt_monitor.pluggedIn():
          self.batt_monitor.sendPluggedEmail()
          self.plugged_email_sent = True
          print "Sent plugged e-mail"
        if self.batt_monitor.chargeDone():
          if(self.batt_monitor.pluggedIn()):
            if not self.unplug_email_sent:
              self.batt_monitor.sendUnPlugEmail()
              self.unplug_email_sent = True
              print "Sent unplug e-mail"
          else:
            self.unplug_email_sent = False
            self.plugged_email_sent = False
            self.batt_monitor.sendUnpluggedEmail()
            print "Sent unplugged e-mail"
            #resume the current goal
            self.navigator.sendGoal(self.current_goal, "/map")
            self.state = "nav"
            print "recharge --> nav"
      elif self.state == "nav_charge":
        """
          State Transitions from nav_charge
          1) nav_charge --- robot has reached charging station ---> recharge
          2) nav_charge --- robot is inactive or timeout is reached --> nav_charge
        """
        if self.navigator.goalReached() and manual_charger.chargeRequested() and not self.batt_monitor.pluggedIn():
          print "nav_charge --> nav_charge"


        elif self.navigator.goalReached():
          manual_charger.reset()
          self.batt_monitor.sendPlugEmail()
          print "Sent plug e-mail"
          self.state = "recharge"
          print "nav_charge --> recharge"
        elif not self.navigator.active() or self.navigator.timeUp():
          chrg_pts = self.chrg_stations[random.randint(0, len(self.chrg_stations) - 1)]
          self.navigator.sendGoal(chrg_pts, "/map")
          print "nav_charge --> nav_charge"
    else:
      if not self.move_head_adapter.legalState():
        print("Waiting on %s to be published" % (self.move_head_adapter.state_topic_))
        rospy.logout("Waiting on %s to be published" % (self.move_head_adapter.state_topic_))

      if not self.move_head_adapter2.legalState():
        print("Waiting on %s to be published" % (self.move_head_adapter2.state_topic_))
        rospy.logout("Waiting on %s to be published" % (self.move_head_adapter2.state_topic_))

      if not self.capture_waiter.legalState():
        print("Waiting on %s to be published" % (self.capture_waiter.state_topic_))
        rospy.logout("Waiting on %s to be published" % (self.capture_waiter.state_topic_))
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

    rospy.init_node("ExecutiveDataCollector", anonymous=True)

    batt_monitor = BatteryMonitorAdapter(.25, .75, "battery_state", ["sorokin@willowgarage.com"], "prg", "/usr/sbin/sendmail")
    navigator = NavigationAdapter(30, 300, "/move_base/feedback", "/move_base/activate")
    unstuck = StuckAdapter("/base_controller/state", "/cmd_vel", 0.5) 
    manual_charger = ManualChargeAdapter("/request_charge")

    #/laser_tilt_controller/laser_scanner_signal - better!
    capture_waiter = WaitForKMessagesAdapter("wait_k_messages_action",35,50)
    
    #capture_configs=[[0.0,-0.1],[-0.5, 0.3],[0.5, 0.3]];

    #multi_config_waiter=WaitForMultipleHeadConfigsAdapter(capture_configs,capture_waiter);

    move_head_adapter=MoveHeadAdapter("/move_head_M/move_head_action",-1);
    move_head_adapter2=MoveHeadAdapter("/move_head_C/move_head_action",-1);

    hc_goal_topic_ = "/head_controller/set_command_array"
    hc_pub = rospy.Publisher(hc_goal_topic_, JointCmd)

    joint_cmds=JointCmd();
    joint_cmds.names=[ "head_pan_joint", "head_tilt_joint"];
    joint_cmds.positions=[0.0,0.3];

    hc_pub.publish(joint_cmds);
    rospy.sleep(0.1);
    hc_pub.publish(joint_cmds);

    chrg_stations = [
    [[18.098, 21.015, 0.000], [0.000, 0.000, 0.713, 0.701]]
    ]

    goals_file=sys.argv[1];
    goals=open(goals_file,'r').readlines()
    
    #goals=annotated_map_builder.target_poses.willow_default_pose_list;
    new_goals=[];
    pi2=3.14/2;
    for g in goals:
      if g.strip()[0]=="#":
        continue
      #(locX,locY,orient) = [float(s) for s in g.strip().split(" ")];
      (locX,locY,locZ,q0,q1,q2,q3) = [float(s) for s in g.strip().split(" ")];
      #for th in [0, pi2, 2*pi2,3*pi2/4]:
      #thNew=orient;
      #qt=tf.transformations.quaternion_about_axis(thNew,[0,0,1]);
      new_goals.append([[locX,locY,locZ],[q0,q1,q2,q3]])
      #new_goals.append([[locX,locY,0],[qt[0],qt[1],qt[2],qt[3]]])
        #thNew=orient+th;
        #while thNew>4*pi2:
        #  thNew-=4*pi2


    executive = ExecutiveDataCollector(new_goals, chrg_stations, navigator, batt_monitor, unstuck, manual_charger, capture_waiter, move_head_adapter,move_head_adapter2, 1.0)
    executive.run()
  except KeyboardInterrupt, e:
    pass
  print "exiting"


