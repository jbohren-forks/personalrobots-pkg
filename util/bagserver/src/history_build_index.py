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

PKG = 'roshist'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord

import pickle


def build_index(index_name,topic_filter,bag_names):
    if topic_filter:
        if topic_filter=="*":
            topic_filter_dict=None;
        else:
            topic_filter_dict={};
            for t in topic_filter.strip().split(','):
                topic_filter_dict[t]=1;
    else:
        topic_filter_dict=None;
            
    all_topics={};
    index={};
    for iBag,bag_name in enumerate(bag_names):
        print iBag,bag_name
        for i,(topic, msg, t) in enumerate(rosrecord.logplayer(bag_name, raw=True)):
            if rospy.is_shutdown():
                break
            if topic_filter_dict:
                if not topic in topic_filter_dict:
                    continue
            s=t.secs
            if i % 100 ==0:
                print "\t",i,s,msg[3]

            ns=t.nsecs
            if s not in index:
                lvl2={};
            else:
                lvl2=index[s];
            if topic not in lvl2:
                topic_list=[];
            else:
                topic_list=lvl2[topic]
            topic_list.append((ns,msg[3],iBag))
            topic_list=sorted(topic_list);
            lvl2[topic]=topic_list;
            index[s]=lvl2;

            if topic not in all_topics:
                all_topics[topic]=(msg[0],msg[2])

        
    fOut=open(index_name,'w');
    pickle.dump((all_topics,index,bag_names),fOut);
    fOut.close();


if __name__ == '__main__':
  import sys
  if len(sys.argv) >= 4:
    build_index(sys.argv[1], sys.argv[2], sys.argv[3:])
  else:
    print "usage: history_build_index.py <index_name> <topic_filter> bag_names"
