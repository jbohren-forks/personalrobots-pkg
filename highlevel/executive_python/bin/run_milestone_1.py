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
import rostools
rostools.update_path('executive_python')
import rospy
from battery_monitor_adapter import *
from recharge_adapter import *
from navigation_adapter import *
from executive import *

if __name__ == '__main__':
  try:
    batt_monitor = BatteryMonitorAdapter(.7, "bogus_battery_state")
    recharger = RechargeAdapter(.8, "recharge_state", "recharge_goal")
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

    executive = Executive(goals, chrg_stations, navigator, batt_monitor, recharger, 1.0)
    executive.run()
  except KeyboardInterrupt, e:
    pass
  print "exiting"
