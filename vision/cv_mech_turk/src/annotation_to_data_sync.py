#!/usr/bin/env python



import roslib; roslib.load_manifest('cv_mech_turk')
import rospy

import os
import sys
import getopt

from math import *


from cv_mech_turk.msg import ExternalAnnotation
from image_msgs.msg import RawStereo
from tf.msg import tfMessage

class AnnotationToDataSync:
  def __init__(self):
    #TODO create proper transform

    self.annotation_count=0;
    self.annotations_by_time={};
    self.stored_stereo_info=None;

    self.a_pub = rospy.Publisher('stereo_objects/annotations_2d', ExternalAnnotation)
    self.stereo_pub = rospy.Publisher('stereo_objects/stereo/raw_stereo', RawStereo)
    self.tf_pub = rospy.Publisher('stereo_objects/tf_message', tfMessage)      
    pass


  def handle_annotation(self, msg):
    s=msg.reference_time.secs
    ns=msg.reference_time.nsecs
    if s not in self.annotations_by_time:
      l=[];
    else:
      l=self.annotations_by_time[s];
    l.append([ns,msg]);
    self.annotations_by_time[s]=l;

    print "A:",msg.reference_time




  def select_best_annotation(self, stamp):
    s=stamp.secs;
    ns=stamp.nsecs;
    if s not in self.annotations_by_time:
      #Nothing at this second
      return None

    l=self.annotations_by_time[s];
    minD=None
    bestA=None
    for iA,(a_ns,a_msg) in enumerate(l):
      d=abs(ns-a_ns);
      if not bestA:
        minD=d;
        bestA=a_msg;
        continue

      if d<minD:
        minD=d;
        bestA=a_msg;

    if minD>0:
      return None
    return bestA


  def handle_raw_stereo(self, stereo_msg):
    
    annotation=self.select_best_annotation(stereo_msg.header.stamp)
    
    if not annotation:
      print "S:",stereo_msg.header.stamp
      return

    if len(annotation.boxes)==0 and len(annotation.polygons)==0:
      #print "Skipping empty annotation:", stereo_msg.header.stamp
      #return
      pass
    

    if annotation.reference_time.nsecs == stereo_msg.header.stamp.nsecs:
      self.annotation_count=self.annotation_count+1;
      #HACK - efficient dumping
      fn="/tmp/%d.%d.pts.tgz" %(annotation.reference_time.secs,annotation.reference_time.nsecs)
      if os.path.exists(fn):
        print "Skipping; File exists: ",fn
        return
      print "+",self.annotation_count
      self.a_pub.publish(annotation)
      self.tf_pub.publish(self.tf_)      
      self.stereo_pub.publish(stereo_msg)


  def handle_tf(self,tfmsg):
    self.tf_=tfmsg;

    

def main(args):
  a_lifter = AnnotationToDataSync()
  rospy.init_node('annotation_sync')
  rospy.Subscriber('annotations_2d', ExternalAnnotation, a_lifter.handle_annotation, queue_size=200, buff_size=70000000);
  rospy.Subscriber('stereo/raw_stereo', RawStereo, a_lifter.handle_raw_stereo, queue_size=2000, buff_size=300000000);
  rospy.Subscriber('tf_message', tfMessage, a_lifter.handle_tf, queue_size=200, buff_size=70000000);
  
  rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
