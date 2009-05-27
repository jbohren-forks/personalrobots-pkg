#!/usr/bin/python


"""
usage: %(progname)s --session=SESSION --server=SERVER [--size=ImgWidthxImgHeight]

  * Loads annotation results from the server and sends them as ROS messages
  * --size=640x480 - will convert annotations to 640x480 frame. By default no conversion is done.
  
"""
import roslib; roslib.load_manifest('cv_mech_turk')
import rospy


import uuid,sys,os, string, time, getopt

import urllib2, cookielib

import mimetypes
import mimetools

import pprint

import xml.dom.minidom
from xml.dom.minidom import Node




from cv_mech_turk.msg import *;

class MechFetchResults:
  def __init__(self, srv_name, target_session, target_image_size,ref_frame="",ref_topic=""):


    self.srv_name = srv_name
    self.target_session = target_session

    self.target_image_size=target_image_size

    self.cj = cookielib.CookieJar()
    self.opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(self.cj))
    
    self.ref_frame=ref_frame
    self.ref_topic=ref_topic

  def convert_xml2image(self,xmltxt,target_resolution):
      s=max(target_resolution)/500.0;
      oX=-max(0,target_resolution[1]-target_resolution[0])/2;
      oY=-max(0,target_resolution[0]-target_resolution[1])/2;

      doc = xml.dom.minidom.parseString(xmltxt)

      for pt in doc.getElementsByTagName("pt"):
          x= float(pt.getAttribute("x"))
          y= float(pt.getAttribute("y"))

          x2 = x*s+oX;
          y2 = y*s+oY;          
          
          pt.setAttribute("x",str(x2))
          pt.setAttribute("y",str(y2))


      for box in doc.getElementsByTagName("bbox"):
          l= float(box.getAttribute("left"))
          t= float(box.getAttribute("top"))
          w= float(box.getAttribute("width"))
          h= float(box.getAttribute("height"))          

          l2 = l*s+oX;
          t2 = t*s+oY;
          w2 = w*s;
          h2 = h*s;                    
          
          box.setAttribute("left",str(l2))
          box.setAttribute("top",str(t2))
          box.setAttribute("width",str(w2))
          box.setAttribute("height",str(h2))

      return doc.toprettyxml();
  def convert_xml2msg(self,response,img):
    doc = xml.dom.minidom.parseString(response)
    
    msg=ExternalAnnotation();
    msg.task="willow-env-4"
    msg.reference_frame=self.ref_frame
    msg.reference_topic=self.ref_topic

    (secs,nsecs)=img.split("-")[-1].split(".")[0:2];
    msg.reference_time.secs=int(secs);
    msg.reference_time.nsecs=int(nsecs);
    msg.header.stamp.secs=msg.reference_time.secs;
    msg.header.stamp.nsecs=msg.reference_time.nsecs+1;
    msg.quality=0.2
    msg.boxes=[]
    for box in doc.getElementsByTagName("bbox"):
      l= float(box.getAttribute("left"))
      t= float(box.getAttribute("top"))
      w= float(box.getAttribute("width"))
      h= float(box.getAttribute("height"))          
      
      boxmsg=AnnotationBox();
      boxmsg.object_name=str(box.getAttribute("name"))
      boxmsg.left=l;
      boxmsg.top=t;
      boxmsg.width=w;
      boxmsg.height=h;
      boxmsg.control_points=[]
      boxmsg.control_points.append(AnnotationPt2D(l,t))
      boxmsg.control_points.append(AnnotationPt2D(l+w,t))
      boxmsg.control_points.append(AnnotationPt2D(l+w,t+h))
      boxmsg.control_points.append(AnnotationPt2D(l,t+h))
      msg.boxes.append(boxmsg)
    msg.polygons=[]
    for poly in doc.getElementsByTagName("polygon"):
      polymsg=AnnotationPolygon();
      polymsg.object_name=str(poly.getAttribute("name"))
      polymsg.control_points=[]
      for pt in poly.getElementsByTagName("pt"):    
        x= float(pt.getAttribute("x"))
        y= float(pt.getAttribute("y"))
        polymsg.control_points.append(AnnotationPt2D(x,y));
      msg.polygons.append(polymsg)
            
    return msg;
    
  def fetch_results(self):

    
    pub = rospy.Publisher('annotations_2d', ExternalAnnotation)
    rospy.init_node('session_2_messages')
    print "Sleep"
    time.sleep(5)
    print "Wake up"


    url= "http://"+self.srv_name + "/mt/session_images2/%s/" % (self.target_session, )

    try:
        print url
        fp = self.opener.open(url)
        response = fp.read()              

    except urllib2.URLError, reason:
        print reason
        return None

    print response
    results=response.strip().split("\n");
    results=map(lambda s:s.split("\t"),results);


    print "Beginning to download %d results" %(len(results))
    nOk=0;
    nFail=0;
    for idx,(img,url) in enumerate(results):
        if idx % 20 ==0:
            print "%d of %d" % (idx,len(results))
        #if idx % 25 ==5:
	#	break

        try:
            fp = urllib2.urlopen(url)
            response = fp.read()
            fp.close();

	    try:
                if self.target_image_size:
                    response=self.convert_xml2image(response,self.target_image_size);
		print img
		msg=self.convert_xml2msg(response,img);
		if msg:
			pub.publish(msg);
        
            
                #fXml=open(full_filename,'w')
                #print >>fXml,response
                #fXml.close()
                nOk=nOk+1
            except ValueError, reason:
                print img,reason
                nFail=nFail+1
        except urllib2.URLError, reason:
            print img,reason
            nFail=nFail+1
        if rospy.is_shutdown():
          break
            
    print "%d annotations retreived, %d failed" % (nOk,nFail)


        
        
        

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug", "session=", "server=", "size=","ref-frame=","ref-topic="])

  testflag = 0

  target_session=None
  srv_name="vm6.willowgarage.com:8080"
  target_size=None
  ref_frame="stereo_l_stereo_camera_frame"
  ref_topic="/stereo/left/image"

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1
    elif field == "--session":
      target_session = val
    elif field == "--server":
      srv_name = val
    elif field == "--size":
      target_size=map(lambda s:int(s),val.split('x'))
    elif field == "--ref-frame":
      ref_frame = val
    elif field == "--ref-topic":
      ref_topic = val

  if testflag:
    test()
    return

  if not target_session:
    usage(progname);
    return


  mech = MechFetchResults(srv_name, target_session, target_size,ref_frame,ref_topic)

  mech.fetch_results()
               

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
