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


def build_request(theurl, fields, files, txheaders=None):
  content_type, body = encode_multipart_formdata(fields, files)
  if not txheaders: txheaders = {}
  txheaders['Content-type'] = content_type
  txheaders['Content-length'] = str(len(body))
  return urllib2.Request(theurl, body, txheaders)

def encode_multipart_formdata(fields, files, BOUNDARY = '-----'+mimetools.choose_boundary()+'-----'):

    """ Encodes fields and files for uploading.

    fields is a sequence of (name, value) elements for regular form fields - or a dictionary.

    files is a sequence of (name, filename, value) elements for data to be uploaded as files.

    Return (content_type, body) ready for urllib2.Request instance

    You can optionally pass in a boundary string to use or we'll let mimetools provide one.

    """    

    CRLF = '\r\n'

    L = []

    if isinstance(fields, dict):
        fields = fields.items()

    for (key, value) in fields:
        L.append('--' + BOUNDARY)
        L.append('Content-Disposition: form-data; name="%s"' % key)
        L.append('')
        L.append(value)

    for (key, filename, value) in files:
        filetype = mimetypes.guess_type(filename)[0] or 'application/octet-stream'
        L.append('--' + BOUNDARY)
        L.append('Content-Disposition: form-data; name="%s"; filename="%s"' % (key, filename))
        L.append('Content-Length: %s' % len(value))
        L.append('Content-Type: %s' % filetype)
        L.append('Content-Transfer-Encoding: binary')
        L.append('')
        L.append(value)

    L.append('--' + BOUNDARY + '--')
    L.append('')
    body = CRLF.join(L)

    content_type = 'multipart/form-data; boundary=%s' % BOUNDARY        # XXX what if no files are encoded

    return content_type, body



class BoxGrouppingSubmitter:
  def __init__(self, srv_name, target_session,  source_session):

    self.srv_name = srv_name
    self.target_session = target_session
    self.source_session = source_session;

    self.cj = cookielib.CookieJar()
    self.opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(self.cj))
    


  def get_xml2server_image_map(self, annotations_dir):
    d={}
    for l in open(os.path.join(annotations_dir,"server_names.txt"),"r").readlines():
      (srv,local)=l.strip().split(' ');
      d[local+".xml"]=srv
    return d

  def get_all_boxes(self, annotations_dir, object_name):
    all_boxes=[];
    files=os.listdir(annotations_dir);
    for fn in files:
      try:
        doc = xml.dom.minidom.parse(os.path.join(annotations_dir,fn))
      except:
        continue
      
      boxes=doc.getElementsByTagName("bbox");
      for box in boxes:
        name=box.getAttribute("name")
        if name <> object_name:
          continue
        all_boxes.append((fn,box))

    return all_boxes
    


  def get_box_URL(self,box,name_mapping):
    fn=box[0];
    box_xml=box[1];
    l= float(box_xml.getAttribute("left"))
    t= float(box_xml.getAttribute("top"))
    w= float(box_xml.getAttribute("width"))
    h= float(box_xml.getAttribute("height"))          
    url=self.srv_name+"datastore/wnd2/"+self.source_session+"/"+name_mapping[fn]+".jpg/%f/%f/%f/%f/" % (l,t,w,h)
    return url

  def convert_boxes_to_tasks(self, boxes, num_per_task, name_mapping):
    all_tasks=[];
    current_task=[];
    for iBox,box in enumerate(boxes):
      if iBox>0 and (iBox % num_per_task==0):
        all_tasks.append(current_task);
        current_task=[];

      current_task.append(self.get_box_URL(box,name_mapping))

    all_tasks.append(current_task);

    return all_tasks
      


  def submit_tasks(self,tasks):

    for t in tasks:
      x_doc=xml.dom.minidom.Document();
      x_root = x_doc.createElement("grouping")
      x_doc.appendChild(x_root);
      for img in t:
        x_info = x_doc.createElement("img")
        x_root.appendChild(x_info);
        x_info.setAttribute("src",img);

      task_xml = x_doc.toprettyxml();
      self.submit(self.target_session,task_xml)

    print "Submitted %d tasks" % len(tasks)

  def submit(self, session,parameters):

    url = self.srv_name+"mt"+ "/new_HIT_generic/"

    fields = []
    fields.append(('session',self.target_session))
    fields.append(('parameters',parameters))

    input = build_request(url, fields, [])
    print input
    try:
      fp = self.opener.open(input)
      response = fp.read()              
    except urllib2.HTTPError, reason:
      body = reason.read()
      open("error.html", "w").write(body)
      raise
      

    print repr(response)

    external_hit_id = response.strip()

    if len(external_hit_id) < 36:
      raise IOError, "invalid external hit id: %s" % external_hit_id
    
    return external_hit_id

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


  def fetch_results(self):
    url= "http://"+self.srv_name + "/mt/session_images3/%s/" % (self.target_session )

    try:
        print url
        fp = urllib2.urlopen(url)
        response = fp.read()              

    except urllib2.URLError, reason:
        print reason
        return None

    results=response.strip().split("\n");
    results=map(lambda s:s.split("\t"),results);

    if not os.path.exists(self.output_dir):
        os.makedirs(self.output_dir)

    print "Starting to download %d results" %(len(results))
    nOk=0;
    nFail=0;
    for idx,(server,annotation_path,session,image_id,image_frame_id,image_time,image_topic) in enumerate(results):
        if idx % 20 ==0:
            print "%d of %d" % (idx,len(results))

        #full_annotation_path=os.path.join(self.output_dir,session);
        full_annotation_path=self.output_dir;
        if not os.path.exists(full_annotation_path):
          os.makedirs(full_annotation_path)

        full_annotation_filename=os.path.join(full_annotation_path,image_time+'.xml');

        #image_output_path=os.path.join(self.image_output_dir,session);
        image_output_path=self.image_output_dir;
        if not os.path.exists(image_output_path):
          os.makedirs(image_output_path);
        image_filename=os.path.join(image_output_path,image_time+'.jpg');

        #Maybe add this later
        #if os.path.exists(full_annotation_filename):
        #    continue

        try:
            if not os.path.exists(image_filename):
              image_url=server+"frames/"+session+"/"+image_id+".jpg";
              print image_filename,image_url
              os.system("wget -O %s %s" % (image_filename,image_url))

            if annotation_path[0]=='/':
              annotation_path=annotation_path[1:]
            url= server + annotation_path
            print url
            fp = urllib2.urlopen(url)
            response = fp.read()
            fp.close();

	    try:
              if self.target_image_size:
                target_size=self.target_image_size
              else:
                im=Image.open(image_filename);
                target_size=im.size;

              response=self.convert_xml2image(response,target_size);
            
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
            
    print "%d annotations retreived, %d failed" % (nOk,nFail)


        
        
        

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug", "target-session=", "source-session=","server=", "annotations=", "object="])

  testflag = 0

  target_session=None
  source_session=None
  srv_name="vm7.willowgarage.com:8080"
  annotations_dir=None
  object=None

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1
    elif field == "--target-session":
      target_session = val
    elif field == "--source-session":
      source_session = val
    elif field == "--server":
      srv_name = val
    elif field == "--annotations":
      annotations_dir = val
    elif field == "--object":
      object_name = val

  if not target_session:
    usage(progname);
    return

  if srv_name[-1] <> '/':
    srv_name += '/';
  if not srv_name.startswith("http://") and not srv_name.startswith("https://"):
    srv_name = 'http://'+srv_name;

  num_per_task=20

  submitter = BoxGrouppingSubmitter(srv_name, target_session,source_session)
  boxes = submitter.get_all_boxes(annotations_dir, object_name);
  name_mapping = submitter.get_xml2server_image_map(annotations_dir)
  tasks = submitter.convert_boxes_to_tasks(boxes, num_per_task,name_mapping)
  submitter.submit_tasks(tasks)
  print tasks

               

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
