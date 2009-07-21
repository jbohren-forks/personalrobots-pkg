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
from bagserver.srv import *

def str2time(s):
    parts=s.strip().split(' ');
    print parts

    sec=int(parts[0]);
    nsec=int(parts[1]);
    return rospy.Time(sec,nsec);    


if __name__ == '__main__':

  rospy.init_node("play_frames_query", anonymous=True)

  fname=rospy.get_param("~frames_file");
  try:
      secBefore=int(rospy.get_param("~secs_before"));
  except:
      secBefore=0;
      
  #fname="selected_frames2"
  q="*"

  svc="hist"
  rospy.wait_for_service(svc)
  hist = rospy.ServiceProxy(svc, History)

  dS=0;
  dNS=1000;
  fIn=open(fname,'r');
  for l in fIn.readlines():
    #(sec,nsec)=[ int(x) for x in ().split(' ')];
    #print sec,nsec
    b=str2time(l.strip());
    e=str2time(l.strip());
    print b
    e=b;
    b.secs -= dS - secBefore;
    e.secs += dS;
    b.nsecs -= dNS;
    e.nsecs += dNS;

    print b,e
    resp = hist(b, e, q)    
    rospy.sleep(1.0)



