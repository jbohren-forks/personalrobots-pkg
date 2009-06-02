#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

PKG = 'bagserver'
import roslib; roslib.load_manifest(PKG)

import rospy
import sys
from bagserver.srv import *


def str2time(s):
    parts=s.strip().split('.');
    if len(parts)==1:
        parts.append("0");
    if len(parts[1])<9:
        parts[1]=parts[1] + "0"*(9-len(parts[1]));
    print parts

    sec=int(parts[0]);
    nsec=int(parts[1]);
    return rospy.Time(sec,nsec);    

def call_history(f,t,svc="hist",q=""):

    rospy.wait_for_service(svc)
    try:
        hist = rospy.ServiceProxy(svc, History)

        b=str2time(f);
        e=str2time(t);
        resp = hist(b, e, q)

        #resp = hist(b, e,"/stereo/raw_stereo")

        #resp = hist(b, e,"/annotations_2d")

        #resp = hist(rospy.Time(0,0), rospy.Time.now(),"/stereo/raw_stereo")
        return "done"
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    if len(sys.argv)==3:
        print call_history(sys.argv[1],sys.argv[2]);
    elif len(sys.argv)==4:
        print call_history(sys.argv[1],sys.argv[2],sys.argv[3]);
    elif len(sys.argv)==5:
        print call_history(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4]);
    else:
        print "Usage: bagserver_query,py timeFrom timeTo [svc_name=hist] [topics=*]"

