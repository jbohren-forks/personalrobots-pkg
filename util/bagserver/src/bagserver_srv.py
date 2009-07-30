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
import roslib.scriptutil
import rosrecord
from copy import copy

import pickle
import os.path

from bagserver.srv import *

from std_msgs.msg import Empty
import roslib.msg

class TimelineInterval:
    def __init__(self,b=None,t=None,e=None):
        self.begin=b;
        self.end=e;
        self.topic=t;
    def __str__(self):
        return "[%d.%d - %d.%d] @ %s" %(self.begin.secs,self.begin.nsecs,self.end.secs,self.end.nsecs,self.topic);

class BagServerSrv:

    def __init__(self):

        self.out_namespace=rospy.get_param("~namespace");
        self.index_name_=rospy.get_param("~index");

        self.setup_hist();

        self.s = rospy.Service('hist', History, self.handle_query)

        self.s_cloud = rospy.Service('fetch_cloud', FetchCloud, self.handle_fetch_cloud)
        self.s_image = rospy.Service('fetch_image', FetchImage, self.handle_fetch_image)
        self.s_annotation = rospy.Service('fetch_annotation', FetchExternalAnnotation, self.handle_fetch_annotation)

        self.s_info = rospy.Service('timeline_info', TimelineInfo, self.handle_timeline_info)
        rospy.loginfo("Bagserver setup done: %s for %s" % (self.out_namespace, self.index_name_))

    def setup_hist(self):
        
        fIn=open(self.index_name_,'r');
        (all_topics,index,bag_names)=pickle.load(fIn);
        self.base_name=os.path.dirname(self.index_name_);

        fIn.close();    
        self.all_topics=all_topics;
        self.bag_names=bag_names;

        self.index=index;
        self.min_t=min(index.keys());
        self.max_t=max(index.keys());


        self.active_topics={};

        self.player_fcn=[];
        self.player=[];
        for iBag,bag_name in enumerate(self.bag_names):
            f,version = rosrecord.open_log_file(os.path.join(self.base_name,bag_name))

            next_msg = {
                rosrecord.HEADER_V1_1 : rosrecord.next_msg_v1_1,
                rosrecord.HEADER_V1_2 : rosrecord.next_msg_v1_2
                }[version];
            self.player_fcn.append(next_msg);
            self.player.append(f);


        self.publishers={};
        
        for topic,(msgType,md5) in self.all_topics.items():
            msg=roslib.scriptutil.get_message_class(msgType);

            if msg is None:
                rospy.logwarn("Missing message type",msgType,"for topic",topic)
                self.publishers[topic]=None;
                continue

            rosrecord.g_message_defs[md5]=msg;

            if topic[0]=='/':
                out_topic=topic;
            else:
                out_topic='/'+topic;

            pub = rospy.Publisher(self.out_namespace+out_topic, msg)
            self.publishers[topic]=pub;

        self.reset_time_pub_ = rospy.Publisher(self.out_namespace+"/reset_time", Empty)
        self.time_pub_ = rospy.Publisher(self.out_namespace+"/time", roslib.msg.Time)

    def pick_next_topic(self):
        if len(self.active_topics)==0:
            return None
        best_topic=min(self.active_topics.values());
        return best_topic[4]

    def advance_topic(self,topic,upper_limit,seed=None):
        if topic in self.active_topics:
            (sec,nsec,idx,file_pos,topic,iBag)=self.active_topics[topic]
        elif seed is None :
            rospy.logwarn("Topic is inactive and no seed is specified.")
            return None
        else:
            sec=max(self.min_t,seed.secs)
            nsec=seed.nsecs
            idx=-1;

        while 1:
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

            if seed and sec==seed.secs:
                if msg_nsec<nsec:
                    continue
            
            self.active_topics[topic]=(sec,msg_nsec,idx,topic_message_list[idx][1],topic,topic_message_list[idx][2])
            return (sec,msg_nsec)


    def setll(self,req): #set lower limit
        if req.begin.secs>self.max_t or req.end.secs<self.min_t:
           return;

        rospy.logdebug("setll - looking for a topic")
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
            rospy.logdebug("setll - seek in topic %s" % topic)

            sec=req.begin.secs;
            nsec=req.begin.nsecs;
            if topic_filter_dict:
                if not topic in topic_filter_dict:
                    continue

            rospy.logdebug("setll - advance topic %s" % topic)
            t=self.advance_topic(topic,req.end,req.begin)
            while t:

                (s,ns)=t
                if s>sec or (s==sec and ns>=nsec):
                    #This topic is at the right position
                    break 
                rospy.loginfo("a" )
                t=self.advance_topic(topic,req.end)
                if not t:
                    if topic in self.active_topics:
                        del self.active_topics[topic];
                    break




        
    def handle_query(self,req):
        rospy.logdebug(" Query %s - %s " % (req.begin,req.end))

        self.setll(req);

        e=Empty();
        self.reset_time_pub_.publish(e);

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
            pub=self.publishers[topic]
            if pub:
                #try:
                (tmp_topic, msg, msg_t)=  self.player_fcn[iBag](self.player[iBag],raw=False)
                pub.publish(msg)
                #except:
                #    print "ERROR deserealizing the message in topic",topic,". Most likely the message has changed."

            sim_time=roslib.msg.Time();
            sim_time.rostime.secs=sec;
            sim_time.rostime.nsecs=nsec;
            self.time_pub_.publish(sim_time)
            rospy.sleep(0.00001)


            self.advance_topic(nextT,req.end)
                

        return HistoryResponse();

    
    def handle_fetch_cloud(self,req):
        resp=FetchCloudResponse();
        return self.handle_fetch_data(req,resp)

    def handle_fetch_image(self,req):
        resp=FetchImageResponse();
        return self.handle_fetch_data(req,resp)

    def handle_fetch_annotation(self,req):
        resp=FetchExternalAnnotationResponse();
        return self.handle_fetch_data(req,resp)

    def handle_fetch_data(self,req,resp):
        rospy.logdebug(" Query %s - %s [%s]" % (req.begin,req.topic, req.locator))

        begin=copy(req.begin);
        end=copy(req.begin);
        end.secs+=1;

        timeline_pos=TimelineInterval(begin,req.topic,end);                  


        self.active_topics={};
        self.setll(timeline_pos);

        nextT=self.pick_next_topic();
        if nextT is None:
            rospy.logerr("No message");
            return None


        (sec,nsec,idx,file_pos,topic,iBag)=self.active_topics[nextT]
        self.player[iBag].seek(file_pos);
        (tmp_topic, msg, msg_t)=  self.player_fcn[iBag](self.player[iBag],raw=False)
        resp.result=msg;

        return resp;

    def handle_timeline_info(self,req):
        resp=TimelineInfoResponse();
        resp.begin.secs = self.min_t
        resp.begin.nsecs=0
        resp.end.secs   =self.max_t+1
        resp.end.nsecs=0
        resp.topics=self.all_topics

        return resp


def start_server():
    rospy.init_node('hist_server')
    h = BagServerSrv();
    rospy.spin()

if __name__ == "__main__":
  import sys

  start_server();

  #if len(sys.argv) == 3:
  #  start_server(sys.argv[1], sys.argv[2])
  #else:
  #  print "usage: bagserver_srv.py <out_namespace> <index_name> "



