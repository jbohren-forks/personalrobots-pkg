#!/usr/bin/env python



import roslib; roslib.load_manifest('cv_mech_turk')
import rospy

from cv_mech_turk.msg import ExternalAnnotation


class AnnotationRepublisher:
  def __init__(self):

    self.a_pub = rospy.Publisher('annotations_2d_repub', ExternalAnnotation)
    self.a_sub = rospy.Subscriber('annotations_2d', ExternalAnnotation, self.handle_annotation, queue_size=200)

  def handle_annotation(self, msg):
    msg.header.frame_id=msg.reference_frame;
    self.a_pub.publish(msg);

    

if __name__ == '__main__':

  rospy.init_node('annotation_sync')
  a = AnnotationRepublisher()
  rospy.spin();




