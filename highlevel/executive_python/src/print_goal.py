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


# this is a helper script for extracting goals (poses) from the map


NAME="print_goal"

import roslib
roslib.load_manifest('executive_python')
import rospy
import random
import sys
from robot_msgs.msg import PoseStamped

class GoalPrinter:
  def __init__(self, goal_topic):
    rospy.Subscriber(goal_topic, PoseStamped, self.update)
    self.goal_topic = goal_topic
    rospy.init_node(NAME, anonymous=True)
    rospy.spin()


  def update(self, goal):
    print "Enter id: "
    id = int(sys.stdin.readline())
    print >> sys.stderr, "\"%d\" : [[%f,%f,%f], [%f,%f,%f,%f]],"%(id, goal.pose.position.x,
            goal.pose.position.y,goal.pose.position.z,
            goal.pose.orientation.x,goal.pose.orientation.y,
            goal.pose.orientation.z,goal.pose.orientation.w)


if __name__ == "__main__":
    gp = GoalPrinter(sys.argv[1])
