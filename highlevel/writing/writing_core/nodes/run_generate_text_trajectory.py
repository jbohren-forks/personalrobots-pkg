#! /usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
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
roslib.load_manifest('writing_core')
import rospy

import time
import sys
import math

from std_msgs.msg import Empty

import pr2_robot_actions.msg

import robot_msgs
import python_actions

# http://local.wasp.uwa.edu.au/~pbourke/dataformats/hershey/

raw_hershey = """
    1  9MWRMNV RRMVV RPSTS
    2 16MWOMOV ROMSMUNUPSQ ROQSQURUUSVOV
    3 11MXVNTMRMPNOPOSPURVTVVU
    4 12MWOMOV ROMRMTNUPUSTURVOV
    5 12MWOMOV ROMUM ROQSQ ROVUV
    6  9MVOMOV ROMUM ROQSQ
    7 15MXVNTMRMPNOPOSPURVTVVUVR RSRVR
    8  9MWOMOV RUMUV ROQUQ
    9  3PTRMRV
   10  7NUSMSTRVPVOTOS
   11  9MWOMOV RUMOS RQQUV
   12  6MVOMOV ROVUV
   13 12LXNMNV RNMRV RVMRV RVMVV
   14  9MWOMOV ROMUV RUMUV
   15 14MXRMPNOPOSPURVSVUUVSVPUNSMRM
   16 10MWOMOV ROMSMUNUQSROR
   17 17MXRMPNOPOSPURVSVUUVSVPUNSMRM RSTVW
   18 13MWOMOV ROMSMUNUQSROR RRRUV
   19 13MWUNSMQMONOOPPTRUSUUSVQVOU
   20  6MWRMRV RNMVM
   21  9MXOMOSPURVSVUUVSVM
   22  6MWNMRV RVMRV
   23 12LXNMPV RRMPV RRMTV RVMTV
   24  6MWOMUV RUMOV
   25  7MWNMRQRV RVMRQ
   26  9MWUMOV ROMUM ROVUV
"""

hershey = {}
for l in raw_hershey.split('\n'):
  if len(l) > 8:
    hershey[int(l[0:5])] = l[8:]

def glyph(cmd, org):

  curves = []
  def unR(c):
    return ord(c) - ord('R')
  def coord(s, org):
    return ((unR(s[0]) + org[0], -unR(s[1]) + org[1]), s[2:])

  l,r = unR(cmd[0]), unR(cmd[1])
  org = (org[0] - l, org[1] + 5)
  pos,cmd = coord(cmd[2:], org)
  curve = [pos]

  while cmd != "":
    if cmd[0:2] == " R":
      if curve != []:
        curves.append(curve)
        curve = []
      pos,cmd = coord(cmd[2:], org)
      curve.append(pos)
    else:
      nxt,cmd = coord(cmd, org)
      curve.append(nxt)

  if curve != []:
    curves.append(curve)
  return (-l + r),curves

def traject(msg):
  trj = []

  org = (0,0)
  for c in msg.upper():
    if c == ' ':
      w = 10
    else:
      w, g = glyph(hershey[ord(c) - ord('@')], org)
      trj += g
    org = (org[0] + w, org[1])

  return trj

class GenerateTextTrajectoryAction(python_actions.Action):

  def execute(self, goal):

    points = [(0, 0, 1)]
    for t in traject(goal.text):
      points += [(t[0][0], t[0][1],1)] + [(x,y,0) for (x,y) in t] + [(t[-1][0], t[-1][1],1)]

    msg = robot_msgs.msg.Path()
    msg.poses = []
    for (x,y,z) in points:
      ps = robot_msgs.msg.PoseStamped()
      ps.pose.position = robot_msgs.msg.Point(x, y, z)
      msg.poses.append(ps)
    self.feedback = msg

    return python_actions.SUCCESS

if __name__ == '__main__':

  try:

    rospy.init_node("generate_text_trajectory")
    w = GenerateTextTrajectoryAction("generate_text_trajectory",
                                     pr2_robot_actions.msg.TextGoal,
                                     pr2_robot_actions.msg.GenerateTextTrajectoryState,
                                     robot_msgs.msg.Path)
    w.run()
    rospy.spin();

  except KeyboardInterrupt, e:

    pass

  print "exiting"
