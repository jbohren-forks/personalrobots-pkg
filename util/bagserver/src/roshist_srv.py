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
import roslib.scriptutil
import rosrecord

import pickle

from roshist.srv import *




class RosHistSrv:

    def __init__(self, out_namespace,index_name):
        self.out_namespace=out_namespace;
        self.index_name_=index_name;

        self.setup_hist();


    def setup_hist(self):
        fIn=open(self.index_name_,'r');
        (all_topics,index,bag_names)=pickle.load(fIn);
        fIn.close();    
        self.all_topics=all_topics;
        self.bag_names=bag_names;
        print all_topics
        self.index=index;
        self.min_t=min(index.keys());
        self.max_t=max(index.keys());
        print self.min_t,self.max_t


        self.player_fcn=[];
        self.player=[];
        for iBag,bag_name in enumerate(self.bag_names):
            f,version = rosrecord.open_log_file(bag_name)
            next_msg = {
                rosrecord.HEADER_V1_1 : rosrecord.next_msg_v1_1,
                rosrecord.HEADER_V1_2 : rosrecord.next_msg_v1_2
                }[version];
            self.player_fcn.append(next_msg);
            self.player.append(f);

        self.publishers={};
        for topic,(msgType,md5) in self.all_topics.items():
            msg=roslib.scriptutil.get_message_class(msgType);
            if topic[0]=='/':
                out_topic=topic;
            else:
                out_topic='/'+topic;

            pub = rospy.Publisher(self.out_namespace+out_topic, msg)
            self.publishers[topic]=pub;



    def pick_next_topic(self):
        if len(self.active_topics)==0:
            return None
        best_topic=min(self.active_topics.values());
        return best_topic[4]

    def advance_topic(self,topic,upper_limit,seed=None):
        if topic in self.active_topics:
            (sec,nsec,idx,file_pos,topic,iBag)=self.active_topics[topic]
        elif seed is None :
            print "no seed" 
            return None
        else:
            sec=max(self.min_t,seed.secs)
            nsec=seed.nsecs
            idx=-1;

        while 1:
            #print sec,idx
            if sec>upper_limit.secs or sec>self.max_t:
                if topic in self.active_topics:
                    del self.active_topics[topic];
                return None
            if not sec in self.index:
                idx = -1
                sec += 1;
                continue
            if not topic in self.index[sec]:
                idx = -1
                sec += 1;
                continue
            
            topic_message_list=self.index[sec][topic];
            idx += 1;
            if idx>=len(topic_message_list):
                idx=-1;
                sec += 1;
                continue;
            if sec==upper_limit.secs and topic_message_list[idx][0]>upper_limit.nsecs:
                if topic in self.active_topics:
                    del self.active_topics[topic];
                return None
            msg_nsec=topic_message_list[idx][0];
            #print ".",msg_nsec,nsec
            if seed and sec==seed.secs:
                if msg_nsec<nsec:
                    continue
            
            self.active_topics[topic]=(sec,msg_nsec,idx,topic_message_list[idx][1],topic,topic_message_list[idx][2])
            return (sec,msg_nsec)


    def setll(self,req): #set lower limit
        if req.begin.secs>self.max_t or req.end.secs<self.min_t:
           return;

        self.active_topics={};

        if not req.topic == "":
            if req.topic =="*":
                topic_filter_dict=None;
            else:
                topic_filter_dict={};
                for t in req.topic.strip().split(','):
                    topic_filter_dict[t]=1;
        else:
            topic_filter_dict=None;

        for topic in self.all_topics.keys():
            sec=req.begin.secs;
            nsec=req.begin.nsecs;
            if topic_filter_dict:
                if not topic in topic_filter_dict:
                    continue


            t=self.advance_topic(topic,req.end,req.begin)

            while t:
                #print t,(sec,nsec)
                (s,ns)=t
                if s>sec or (s==sec and ns>=nsec):
                    #This topic is at the right position
                    break 

                t=self.advance_topic(topic,req.end)
                if not t:
                    if topic in self.active_topics:
                        del self.active_topics[topic];
                    break

        print self.active_topics


        
    def handle_query(self,req):
        print req.begin
        print req.end

        self.setll(req);

        
        while 1:
            if rospy.is_shutdown():
                break
            nextT=self.pick_next_topic();
            if nextT is None:
                break
            (sec,nsec,idx,file_pos,topic,iBag)=self.active_topics[nextT]
            #SEND msg
            self.player[iBag].seek(file_pos);
            #(tmp_topic, (datatype, message_data, md5sum, bag_pos), msg_t)=  self.player_fcn(self.player,raw=True)
            (tmp_topic, msg, msg_t)=  self.player_fcn[iBag](self.player[iBag],raw=False)
            self.publishers[topic].publish(msg)
            #print self.active_topics

            self.advance_topic(nextT,req.end)
                

        return HistoryResponse();

    
        

def start_server(out_namespace,index_fn):
    rospy.init_node('hist_server')
    h = RosHistSrv(out_namespace,index_fn);
    s = rospy.Service('hist', History, h.handle_query)
    rospy.spin()

if __name__ == "__main__":
  import sys
  if len(sys.argv) == 3:
    start_server(sys.argv[1], sys.argv[2])
  else:
    print "usage: roshist_srv.py <out_namespace> <index_name> "



