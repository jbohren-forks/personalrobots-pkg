#!/usr/bin/python

"""
usage: %(progname)s --session=SESSION --server=SERVER [filenames...]

  * submits an image to mechanical turk
"""

import uuid,sys,os, string, time, getopt

import urllib2, cookielib

import mimetypes
import mimetools



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


class MechSubmiter:
  def __init__(self, srv_name, target_session):

    self.srv_name = srv_name
    self.target_session = target_session

    self.cj = cookielib.CookieJar()
    self.opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(self.cj))
    
    self.submit_url="http://%s/mt" % (self.srv_name, )

  def submit(self, inp_img):
    target_img_name=uuid.uuid4();

    url = self.submit_url + "/newHIT2/"

    fields = []
    fields.append(('session',self.target_session))
    fields.append(('frame',str(target_img_name)))
    fields.append(('original_name',str(inp_img)))

    attachment=open(inp_img,'rb').read();

    files = []
    files.append(("image", str(target_img_name)+".jpg", attachment))

    input = build_request(url, fields, files)
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

  def check(self, ext_id):
    url= self.submit_url + "/hit_results_xml/%s/" % (ext_id, )

    input = build_request(url, [], [])
    try:
      fp = urllib2.urlopen(url)
      response = fp.read()              
    except urllib2.URLError, reason:
      return None

    return response

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug", "ext_id=", "session="])

  testflag = 0

  ext_id = None
  #target_session="pick-up-2new-demo1-sandbox"
  target_session=None
  srv_name="vm6.willowgarage.com:8080"

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1
    elif field == "--ext_id":
      ext_id = val
    elif field == "--session":
      target_session = val
    elif field == "--server":
      srv_name = val

  if testflag:
    test()
    return

  if not target_session:
    usage(progname);
    return


  if not ext_id and len(args) == 0:
    usage(progname)
    return

  mech = MechSubmiter(srv_name, target_session)

  ext_id_list = []
  if ext_id:
    ext_id_list.append(ext_id)

  for fn in args:
    ext_id = mech.submit(fn)
    print fn, ext_id
    ext_id_list.append((ext_id, fn))

  for (ext_id, fn) in ext_id_list:
    while 1:
      response = mech.check(ext_id)
      if response: break

      time.sleep(5)

    print fn
    print response
      
               

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
