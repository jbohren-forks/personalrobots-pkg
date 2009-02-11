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


class MechFetchResults:
  def __init__(self, srv_name, target_session, output_dir, target_image_size):


    self.srv_name = srv_name
    self.target_session = target_session
    self.output_dir = output_dir

    self.target_image_size=target_image_size

    self.cj = cookielib.CookieJar()
    self.opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(self.cj))
    


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
    url= "http://"+self.srv_name + "/mt/session_images/%s/" % (self.target_session, )

    #input = build_request(url, [], [])
    try:
        print url
        fp = urllib2.urlopen(url)
        response = fp.read()              
        #print response
    except urllib2.URLError, reason:
        print reason
        return None

    results=response.strip().split("\n");
    results=map(lambda s:s.split("\t"),results);

    if not os.path.exists(self.output_dir):
        os.makedirs(self.output_dir)

    print "Beginning to download %d results" %(len(results))
    nOk=0;
    nFail=0;
    for idx,(img,url) in enumerate(results):
        if idx % 20 ==0:
            print "%d of %d" % (idx,len(results))

        fName=img.replace(".jpg",".xml").replace("/","__").replace("\\","__");
        full_filename=os.path.join(self.output_dir,fName);

        #Maybe add this later
        #if os.path.exists(full_filename):
        #    continue

        
        try:
            fp = urllib2.urlopen(url)
            response = fp.read()
            fp.close();

	    try:
                if self.target_image_size:
                    response=self.convert_xml2image(response,self.target_image_size);
            
                fXml=open(full_filename,'w')
                print >>fXml,response
                fXml.close()
                nOk=nOk+1
            except ValueError, reason:
                print img,reason
                nFail=nFail+1
        except urllib2.URLError, reason:
            print img,reason
            nFail=nFail+1
            
    print "%d annotations retreived, %d failed" % (nOk,nFail)


        
        
        

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug", "session=", "server=", "saveto=", "size="])

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

  if testflag:
    test()
    return

  if not target_session:
    usage(progname);
    return

  if not output_folder:
    output_folder=os.path.join("results",target_session);

  mech = MechFetchResults(srv_name, target_session, output_folder, target_size)

  mech.fetch_results()

      
               

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
