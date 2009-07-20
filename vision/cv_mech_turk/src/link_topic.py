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
import roslib.scriptutil
roslib.load_manifest('cv_mech_turk')
import rospy
import random
import sys
from cv_mech_turk.msg import ExternalAnnotation

import urllib2, cookielib

class TopicLinkNode:
  def __init__(self,topic_name,remote_node,remote_publisher):
    self.topic=topic_name
    self.remote_node=remote_node
    self.remote_publisher_uri=remote_publisher

  def link(self):
    m=roslib.scriptutil.get_master();
    m.registerPublisher(self.remote_node,self.topic,'cv_mech_turk/ExternalAnnotation',self.remote_publisher_uri);
    
  def spin(self):
    while True: #not rospy.is_shutdown():
      self.link();
      rospy.sleep(10.0);

class HttpTopicLinkNode:
  def __init__(self):
    try:
      print "FIX get_param!!!"
      remote_publisher_uri=rospy.get_param("~uri","http://vm6.willowgarage.com:8080/mt/rospublishers/");
    except:
      remote_publisher_uri="http://vm6.willowgarage.com:8080/mt/rospublishers/"

    self.ref=remote_publisher_uri

  def link(self):
    self.cj = cookielib.CookieJar()
    self.opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(self.cj))
    fp = self.opener.open(self.ref)
    response = fp.read()              
    if response=="none":
      print "No connections"
      return

    self.links=[];
    for s in response.split('\n'):
      print "Adding connection",s.strip() 
      (n,t,u)=s.strip().split(',');
      l=TopicLinkNode(t,n,u)
      self.links.append(l)
      l.link();

  def spin(self):
    while not rospy.is_shutdown():
      self.link()
      rospy.sleep(10)
    

if __name__ == '__main__':

  rospy.init_node("topic_link", anonymous=True)

  tl=HttpTopicLinkNode()
  tl.spin();

    
