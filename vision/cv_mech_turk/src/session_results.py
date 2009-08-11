#!/usr/bin/python

## Alexander Sorokin
##

"""
usage: %(progname)s --session=SESSION --server=SERVER [--saveto=results/session] [--size=ImgWidthxImgHeight]

  * Loads annotation results from the server and saves them into files locally
  * --size=640x480 - will convert annotations to 640x480 frame. By default no conversion is done.
  
"""

import uuid,sys,os, string, time, getopt

import urllib2, cookielib

import mimetypes
import mimetools
import Image

import pprint

import xml.dom.minidom
from xml.dom.minidom import Node


# Helper function to parse
# <size><width>640</width><height>480</height></size>
def parseSizeTag(size_tag):
    w = float(size_tag.getElementsByTagName("width")[0].childNodes[0].data)
    h = float(size_tag.getElementsByTagName("height")[0].childNodes[0].data)
    return [w,h]


# A patch of size <resolution> whose top left pixel had coordinates <origin>
# was moved and resized to fit a 500x500 window. The results from Mechanical
# Turk are given in this 500x500 window, we want them back in the
# original coordinates.
# This function is recursive because some annotations have another level
# where a subpatch is extracted and resized. At the top level, the <origin>
# of the image should be (0,0), but it is not true further down the tree.
# Currently MTurk only returns two-level annotations, but this function
# should work with more levels as well.
def convert_resolution(node, origin, resolution):
    if node.nodeType != node.ELEMENT_NODE:
        return node

    if origin==None:
        origin = [0,0]
    if resolution==None:
        for child in node.childNodes:
            if child.nodeType == node.ELEMENT_NODE:
                if child.tagName == "size":
                    resolution = parseSizeTag(child)

    if (node.tagName == "bbox") or (node.tagName == "pt"):
        # This node needs to know the resolution. It had better
        # have been found by this level of the tree
        s = max(resolution)/500.0
        oX = -max(0, resolution[1]-resolution[0])/2 + origin[0]
        oY = -max(0, resolution[0]-resolution[1])/2 + origin[1]

        if node.tagName == "bbox":
            # Update the coordinates of this box:
            l = float(node.getAttribute("left"))
            t = float(node.getAttribute("top"))
            w = float(node.getAttribute("width"))
            h = float(node.getAttribute("height"))          
            l2 = l*s + oX
            t2 = t*s + oY
            w2 = w*s
            h2 = h*s                    
            node.setAttribute("left",str(l2))
            node.setAttribute("top",str(t2))
            node.setAttribute("width",str(w2))
            node.setAttribute("height",str(h2))

            # Describe in global coordinates the patch that child annotations
            # are expressed relative to. Use it to convert the child annotations
            # to global coordinates.
            local_origin = [l2, t2]
            local_resolution = [w2, h2]
            for child in node.childNodes:
                if child.nodeType == node.ELEMENT_NODE:
                    if child.tagName == "annotation":
                        convert_resolution(child, local_origin, local_resolution)
                    elif child.tagName == "pt":
                        convert_resolution(child, origin, resolution)
        else:  # node.tagName == "pt"
            x = float(node.getAttribute("x"))
            y = float(node.getAttribute("y"))
            node.setAttribute("x", str(x*s + oX))
            node.setAttribute("y", str(y*s + oY))
    else:
        for child in node.childNodes:
            convert_resolution(child, origin, resolution)



class MechFetchResults:
  def __init__(self, srv_name, target_session, output_dir, image_output_dir=None,target_image_size=None):

    self.srv_name = srv_name
    self.target_session = target_session
    self.output_dir = output_dir

    self.target_image_size=target_image_size

    self.image_output_dir=image_output_dir

    self.cj = cookielib.CookieJar()
    self.opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(self.cj))
    

  # We pass the target resolution as an argument for backwards
  # compatibility, but recent output from Mechanical Turk contains
  # a <size> tag containing the resolution.
  def convert_xml2image(self,xmltxt,target_resolution):
      doc = xml.dom.minidom.parseString(xmltxt)
      convert_resolution(doc.documentElement, [0,0], target_resolution)
      return doc.toxml()

##      s=max(target_resolution)/500.0
##      oX=-max(0,target_resolution[1]-target_resolution[0])/2
##      oY=-max(0,target_resolution[0]-target_resolution[1])/2
##      for pt in doc.getElementsByTagName("pt"):
##          x= float(pt.getAttribute("x"))
##          y= float(pt.getAttribute("y"))
##          x2 = x*s+oX
##          y2 = y*s+oY          
##          pt.setAttribute("x",str(x2))
##          pt.setAttribute("y",str(y2))
##      for box in doc.getElementsByTagName("bbox"):
##          l= float(box.getAttribute("left"))
##          t= float(box.getAttribute("top"))
##          w= float(box.getAttribute("width"))
##          h= float(box.getAttribute("height"))          
##          l2 = l*s+oX
##          t2 = t*s+oY
##          w2 = w*s
##          h2 = h*s                    
##          box.setAttribute("left",str(l2))
##          box.setAttribute("top",str(t2))
##          box.setAttribute("width",str(w2))
##          box.setAttribute("height",str(h2))



  def fetch_results(self):
    url= "http://"+self.srv_name + "/mt/session_images3/%s/" % (self.target_session )

    try:
        print url
        fp = urllib2.urlopen(url)
        response = fp.read()              

    except urllib2.URLError, reason:
        print reason
        return None

    results=response.strip().split("\n")
    results=map(lambda s:s.split("\t"),results)

    if not os.path.exists(self.output_dir):
        os.makedirs(self.output_dir)

    full_annotation_path=self.output_dir
    if not os.path.exists(full_annotation_path):
      os.makedirs(full_annotation_path)
    
    server_image_names_fn=os.path.join(full_annotation_path,'server_names.txt')
    server_names=open(server_image_names_fn,'w')



    print "Starting to download %d results" %(len(results))
    nOk=0
    nFail=0
    for idx,(server,annotation_path,session,image_id,image_frame_id,image_time,image_topic,original_name) in enumerate(results):
        if idx % 20 ==0:
            print "%d of %d" % (idx,len(results))

            
        if image_time=="n/a" or image_time=="0.0":
          image_name=original_name.split("/")[-1]
          name= ".".join(image_name.split(".")[0:-1])
        else:
          name=image_time

        print >>server_names,image_id,name
        print image_id,name
        #full_annotation_path=os.path.join(self.output_dir,session)

        full_annotation_filename=os.path.join(full_annotation_path,name+'.xml')

        #image_output_path=os.path.join(self.image_output_dir,session)
        image_output_path=self.image_output_dir
        if not os.path.exists(image_output_path):
          os.makedirs(image_output_path)
        image_filename=os.path.join(image_output_path,name+'.jpg')

        #Maybe add this later
        #if os.path.exists(full_annotation_filename):
        #    continue

        try:
            if not os.path.exists(image_filename):
              image_url=server+"frames/"+session+"/"+image_id+".jpg"
              print image_filename,image_url
              os.system("wget -O %s %s" % (image_filename,image_url))

            if annotation_path[0]=='/':
              annotation_path=annotation_path[1:]
            url= server + annotation_path
            print url
            fp = urllib2.urlopen(url)
            response = fp.read()
            fp.close()
            # TEMPORARY HACK: REMOVE THE FIRST TAG, WHICH IS CURRENTLY MALFORMED
            # WHEN RETURNED FROM THE SERVER
            response = response[response.index('>')+1:]

	    try:
              if self.target_image_size:
                target_silze=self.target_image_size
              else:
                im=Image.open(image_filename)
                target_size=im.size

              response=self.convert_xml2image(response,target_size)
            
              fXml=open(full_annotation_filename,'w')
              print >>fXml,response
              fXml.close()
              nOk=nOk+1
            except ValueError, reason:
              print image_id,reason
              nFail=nFail+1
        except urllib2.URLError, reason:
          print image_id,reason
          nFail=nFail+1

    server_names.close()
    print "%d annotations retreived, %d failed" % (nOk,nFail)


        
        
        

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug", "session=", "server=", "saveto=", "size=","truesize"])

  testflag = 0

  target_session=None
  srv_name="vm6.willowgarage.com:8080"
  output_folder=None
  target_size=None

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
    elif field == "--saveto":
      output_folder = val
    elif field == "--size":
      target_size=map(lambda s:int(s),val.split('x'))
    elif field == "--truesize":
      target_size="TRUESIZE"

  if testflag:
    test()
    return

  if not target_session:
    usage(progname)
    return

  if not output_folder:
    output_folder=os.path.join("results",target_session)

  annotations_output_folder=os.path.join(output_folder,"annotations")  
  images_output_folder=os.path.join(output_folder,"images")  

  mech = MechFetchResults(srv_name, target_session, annotations_output_folder, images_output_folder)

  mech.fetch_results()

      
               

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
  #doc = xml.dom.minidom.parse("dummy.xml")
  #print doc.toxml()
  #convert_resolution(doc.documentElement, None, None)
  #print doc.toxml()
