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

PKG = 'object_names'
import roslib; roslib.load_manifest(PKG)
import rospy

from object_names.srv import *


class SimpleColorServer:

    def __init__(self):

        self.index_name=rospy.get_param("~index_name")
        
        self.parse_index();
        
        self.s_n2c = rospy.Service('name_to_color', Name2Color, self.handle_name_to_color)


    def parse_index(self):
        fIndex=open(self.index_name,'r');
        self.name2color={};

        for l in fIndex.readlines():
            stripped_line=l.strip();
            if len(stripped_line)==0:
                continue
            if stripped_line[0]=="#":
                continue

            non_empty_parts=filter(lambda s:len(s)>0,stripped_line.split(' '));
            if len(non_empty_parts)!=4:
                rospy.logerr("Can't parse line: %s",l)

            (r,g,b,name)=non_empty_parts
            self.name2color[name]=[float(r)/255.0, float(g)/255.0, float(b)/255.0];

        fIndex.close();



    def handle_name_to_color(self,req):
        resp=Name2ColorResponse();
        if req.name not in self.name2color:
            rospy.logwarn("Missing id for name %s" % req.name) 
            resp.color.r=0.0;
            resp.color.g=0.0;
            resp.color.b=0.0;
            resp.color.a=1.0;
        else:
            (r,g,b)=self.name2color[req.name];
            resp.color.r=r;
            resp.color.g=g;
            resp.color.b=b;
            resp.color.a=1.0;
            pass
        return resp        

        

def start_server():
    rospy.init_node('color_server')
    srv = SimpleColorServer();
    rospy.spin()

if __name__ == "__main__":
  import sys

  start_server();


