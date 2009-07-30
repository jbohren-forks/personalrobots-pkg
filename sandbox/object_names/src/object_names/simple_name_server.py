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


class SimpleNameServer:

    def __init__(self):

        self.index_name=rospy.get_param("~index_name")
        
        self.parse_index();
        
        self.s_n2f = rospy.Service('name_to_float', Name2Float, self.handle_name_to_float)
        self.s_f2n = rospy.Service('float_to_name', Float2Name, self.handle_float_to_name)

    def parse_index(self):
        fIndex=open(self.index_name,'r');
        self.name2id={};
        self.id2name={};
        for l in fIndex.readlines():
            stripped_line=l.strip();
            if len(stripped_line)==0:
                continue
            if stripped_line[0]=="#":
                continue

            non_empty_parts=filter(lambda s:len(s)>0,stripped_line.split(' '));
            if len(non_empty_parts)!=2:
                rospy.logerr("Can't parse line: %s",l)

            (id,name)=non_empty_parts
            self.name2id[name]=float(id);
            int_id=int(round(float(id)));
            if int_id in self.id2name:
                rospy.logerr("ID %d already exists. Note that IDs must be unique after rounding.",id)
                continue
            self.id2name[int_id]=name;
        fIndex.close();



    def handle_name_to_float(self,req):
        resp=Name2FloatResponse();
        if req.name not in self.name2id:
            rospy.logwarn("Missing id for name %s" % req.name) 
            resp.id=0.0;
        else:
            resp.id=self.name2id[req.name];
            pass
        return resp        

    def handle_float_to_name(self,req):
        rospy.loginfo("Translating float %d to name" % req.id) 
        resp=Float2NameResponse();

        int_id=int(round(req.id));
        if int_id not in self.id2name:
            rospy.logwarn("Missing name for id %d (int_id %d)" % (req.id,int_id))
            resp.name="";
        else:
            resp.name=self.id2name[int_id];

        return resp        

        

def start_server():
    rospy.init_node('name_server')
    srv = SimpleNameServer();
    rospy.spin()

if __name__ == "__main__":
  import sys

  start_server();


