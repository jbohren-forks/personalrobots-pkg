#!/usr/bin/env python

import cStringIO
import time
import Image, ImageDraw
import os
import sys
from SocketServer import ThreadingMixIn
import BaseHTTPServer, socket, SocketServer, urlparse, select
import Cookie
import cgi
import threading
import base64
import StringIO
from random import randint as rint

import roslib; roslib.load_manifest('webteleop')
import roslib.scriptutil
import rospy
import rosjson
from tf import *
import bullet

from tf.msg import tfMessage
from nav_srvs.srv import *
from nav_msgs.msg import *
from std_msgs.msg import *
from robot_msgs.msg import *

tfclient = ''
CALLER_ID = '/webteleop'
index = 0

msgClasses = { '/initialpose' : robot_msgs.msg.PoseWithCovariance,
               '/move_base/activate' : robot_msgs.msg.PoseStamped
             }

################################################################################
# Convert an image message to a json message
def img2json(msg, topic_type):
  for dim in msg.uint8_data.layout.dim:
    if dim.label == "width":
      width = dim.size
    elif dim.label == "height":
      height = dim.size

  if topic_type == "sensor_msgs/CompressedImage":
    data = msg.uint8_data.data
  elif topic_type == "sensor_msgs/Image": 
    if msg.encoding == "mono":
      encoding = "L"
    elif msg.encoding == "rgb":
      encoding = "RGB"

    encoding = "L"

    img = Image.frombuffer(encoding, (width, height), msg.uint8_data.data )
    img = img.transpose(Image.FLIP_TOP_BOTTOM)

    buf= StringIO.StringIO()
    img.save(buf, format= 'JPEG')
    data = buf.getvalue()

  data = base64.b64encode(data)
  msg = '{'
  msg += ' "width"  : "%d",' % width
  msg += ' "height" : "%d",' % height
  msg += ' "data" : "data:image/jpeg;base64,' + data + '"'
  msg += '}'

  return msg
#-------------------------------------------------------------------------------

class WebException(Exception): pass



################################################################################
# Ros Web Topic Factory
class RWTFactory(object):
  def __init__(self):
    self.map = {}
    self.lock = threading.Lock()

  def get(self, topic):
    try:
      self.lock.acquire()
      if topic in self.map:
        return self.map[topic]
      self.map[topic] = t = ROSWebTopic(topic)
    finally:
      self.lock.release()
    return t
#*******************************************************************************


################################################################################
# Ros Web TF Factory
class RWTFFactory(object):
  def __init__(self):
    self.map = {}
    self.lock = threading.Lock()

  def get(self, tfname):
    try:
      self.lock.acquire()
      if tfname in self.map:
        return self.map[tfname]
      self.map[tfname] = t = ROSWebTF(tfname)
    finally:
      self.lock.release()
    return t
#*******************************************************************************

factory = RWTFactory()
tffactory = RWTFFactory()


################################################################################
# Ros Web Topic
class ROSWebTopic(object):
  def __init__(self, topic):
    self.topic = topic
    self.topic_type = ""
    self.cond = threading.Condition()
    self.initialized = False
    self.pub = None
    self.sub = None
    self.last_message = ""
    self.pubsub = "subscriber"

  ##############################################################################
  # Init the topic subscriber
  def init(self, _pubsub):
    try:
      self.cond.acquire()

      if self.initialized:
        return

      self.pubsub = _pubsub

      m = roslib.scriptutil.get_master()
      code, _, topics = m.getPublishedTopics(CALLER_ID, '/')

      if code != 1:
        raise WebException("unable to communicate with master")

      for t, self.topic_type in topics:
        if t == self.topic:
          break;


      if self.pubsub == "subscriber":
        msg_class = roslib.scriptutil.get_message_class(self.topic_type)
        self.sub = rospy.Subscriber(self.topic, msg_class, self.topic_callback)
      elif msgClasses.has_key(self.topic):
        self.pub = rospy.Publisher(self.topic, msgClasses[self.topic] )
      else:
        rospy.signal_shutdown("Unknown msg class for topic[%s]" % self.topic)

      self.initialized = True
    except WebException:
      raise
    except Exception, e:
      rospy.signal_shutdown(str(e))
      raise
    finally:
      self.cond.release()
  #-----------------------------------------------------------------------------

  ##############################################################################
  # Unsubscribe from a topic
  def unsubscribe(self):
    self.cond.acquire()
    self.sub.unregister()
    self.pub.unregister()
    self.initialized = False
    self.cond.release()
  #-----------------------------------------------------------------------------

  ##############################################################################
  # Data Callback
  def topic_callback(self, data):
    try:
      self.cond.acquire()
      self.last_message = data
      self.cond.notifyAll()
    finally:
      self.cond.release()

  ##############################################################################
  # Publish a message
  def publish(self, msg):
    try:
      self.cond.acquire()

      if not self.initialized:
        return

      self.pub.publish(msg)

    finally:
      self.cond.release()

    return
  #-----------------------------------------------------------------------------

  ##############################################################################
  # Get data from this topic
  def getData(self):
    msg = ''
    try:
      self.cond.acquire()

      if not self.initialized:
        return msg

      self.cond.wait()

      if self.topic_type == "sensor_msgs/CompressedImage" or self.topic_type == "sensor_msgs/Image": 
        msg = img2json(self.last_message, self.topic_type)

      elif self.topic_type == "visualization_msgs/Polyline":
        frameId = self.last_message.header.frame_id
        msg = '{ "points" : ['
        for i in range( len(self.last_message.points) ):
          point = robot_msgs.msg.PointStamped()
          point.header.stamp = self.last_message.header.stamp
          point.header.frame_id = frameId
          point.point.x = self.last_message.points[i].x
          point.point.y = self.last_message.points[i].y
          point.point.z = self.last_message.points[i].z

          point = tfclient.transformPoint("/map", point)
          msg += '{"x": "%f",' % point.point.x
          msg += '"y": "%f",' % point.point.y
          msg += '"z": "%f"},' % point.point.z

        msg += ']}'

      else:
        msg = rosjson.ros_message_to_json(self.last_message)

    finally:
      self.cond.release()

    return msg
  #-----------------------------------------------------------------------------
#*******************************************************************************




################################################################################
# Ros Web TF
class ROSWebTF():
  def __init__(self, _tfname):
    #self.tl = TransformListener()
    #dur = roslib.rostime.Duration.from_seconds(1.0)
    #self.tl.setExtrapolationLimit(dur)
    self.tfname = _tfname

  def getData(self):
    msg = ''
    try:
      if tfclient.frameExists(self.tfname) and tfclient.frameExists("/map") :
        t = tfclient.getLatestCommonTime("/map", self.tfname )
        position, orientation = tfclient.lookupTransform("/map", self.tfname, t)

        msg = {'position' : 
               {'x': position[0], 'y' : position[1], 'z':position[2]}, 
               'orientation' : 
                 {'x' : orientation[0], 'y': orientation[1], 
                  'z' : orientation[2], 'w': orientation[3]
                 }
              }

    except Exception, e:
      print "Exception: %s" % e

    return msg
#*******************************************************************************

## Utility for connecting to a service and retrieving the TCPROS
## headers. Services currently do not declare their type with the
## master, so instead we probe the service for its headers.
## @param service_name str: name of service
## @param service_uri str: ROSRPC URI of service
## @return dict: map of header fields
## @throws ROSServiceIOException
def get_service_class(service_name):
  master = roslib.scriptutil.get_master()
  code, msg, service_uri = master.lookupService(CALLER_ID, service_name)

  dest_addr, dest_port = rospy.parse_rosrpc_uri(service_uri)
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  try:
      try:
          # connect to service and probe it to get the headers
          s.settimeout(5.0)
          s.connect((dest_addr, dest_port))
          header = { 'probe':'1', 'md5sum':'*',
                     'callerid':'/rosservice', 'service':service_name}
          roslib.network.write_ros_handshake_header(s, header)
          return roslib.network.read_ros_handshake_header(s, cStringIO.StringIO(), 2048).get('type', None)
      except socket.error:
          raise ROSServiceIOException("Unable to communicate with service [%s], address [%s]"%(service_name, service_uri))
  finally:
      if s is not None:
          s.close()

def GetMapAsJPEG(service_proxy, qdict):
  data = ''
  try:
    #staticMap = rospy.ServiceProxy(topic, service_class)
    #staticMap = rospy.ServiceProxy('/static_map', StaticMap)
    map = service_proxy()

    mapW = map.map.info.width
    mapH = map.map.info.height
    data = map.map.data
  except rospy.ServiceException, e:
    print "Service call exception: %s" % e

  originX = 0
  originY = 0
  destW = mapW
  destH = mapH

  if qdict.has_key("x"):
    originX = int(qdict["x"][0])
  if qdict.has_key("y"):
    originY = int(qdict["y"][0])
  if qdict.has_key("w"):
    destW = int(qdict["w"][0])
  if qdict.has_key("h"):
    destH = int(qdict["h"][0])

  img = Image.new("L", (destW,destH), "#FFFFFF")
  draw = ImageDraw.Draw(img)

  y =  0
  for mapy in range(originY, originY+ destH):
    x = 0
    for mapx in range(originX, originX + destW):
      if (ord(data[mapy*mapW+mapx]) < 100):
        draw.point( (x,destH-y-1), fill=(255))
      else:
        draw.point((x,destH-y-1), fill=(0))
      x += 1
    y += 1

  buf= StringIO.StringIO()
  img.save(buf, format= 'JPEG')
  jpeg = buf.getvalue()

  jpeg = base64.b64encode(jpeg)

  msg = '{'
  msg += ' "width"  : "%d",' % destW
  msg += ' "height" : "%d",' % destH
  msg += ' "data" : "data:image/jpeg;base64,' + jpeg + '"'
  msg += '}'

  return msg


################################################################################
# HTTP request handler
class Handler(BaseHTTPServer.BaseHTTPRequestHandler):
  def __init__(self, *args):
    BaseHTTPServer.BaseHTTPRequestHandler.__init__(self, *args)

  ##############################################################################
  # Connect to a server
  def _connect_to(self, netloc, soc):
    print netloc
    i = netloc.find(':')

    if i>=0:
      host_port = netloc[:i], int(netloc[i+1:])
    else:
      host_port = netloc, 80

    try: 
      soc.connect(host_port)
    except socket.error, arg:
      try: 
        msg = arg[1]
      except:
        msg = arg
        self.send_error(404, msg)
        return 0
    return 1
  #-----------------------------------------------------------------------------

  ##############################################################################
  def handleROS(self, path, qdict):
    global factory

    path_parts = path.split('/')

    # Get the command
    cmd = path_parts[2].lower()
    topic = '/'+"/".join( path_parts[p] for p in range(3, len(path_parts) ) )

    print "CMD[%s] Topic[%s]" % (cmd, topic)

    if cmd == "subscribe":
      print "Subscribe[%s]\n" % topic

      rwt = factory.get(topic)

      try:
        rwt.init("subscriber")
      except WebException, e:
        traceback.print_exc()
        self.send_response(404)
        return

      self.send_response(200)
      self.send_header( "Content-type", "text/html" )
      self.end_headers()
      self.wfile.write("subscribed")

    elif cmd == "announce":
      print "Announce[%s]\n" % topic

      rwt = factory.get(topic)

      try:
        rwt.init("publisher")
      except WebException, e:
        traceback.print_exc()
        self.send_response(404)
        return

      self.send_response(200)
      self.send_header( "Content-type", "text/html" )
      self.end_headers()
      self.wfile.write("announced")

    elif cmd == "tfsub":
      print "TF Subscribe[%s]" % topic
      tfmsg = tffactory.get(topic)

      self.send_response(200)
      self.send_header( "Content-type", "text/html" )
      self.end_headers()
      self.wfile.write("subscribed to tf")

    elif cmd == "tfget":
      print "TF GET[%s]" % topic
      tfmsg = tffactory.get(topic)

      msg = tfmsg.getData()

      self.send_response(200)
      self.send_header( "Content-type", "application/json" )
      self.send_header( "Content-length", str(len(msg)) )
      self.end_headers()
      self.wfile.write(msg)

    elif cmd == "unsubscribe":
      print "Unsubscribe[%s]" % topic
      rwt = factory.get(topic)
      rwt.unsubscribe()

      self.send_response(200)
      self.send_header( "Content-type", "text/html" )
      self.end_headers()
      self.wfile.write("unsubscribed")

    elif cmd == "get":

      rwt = factory.get(topic)
      msg = rwt.getData()

      print "Get[%s]" % topic
      self.send_response(200)
      self.send_header( "Content-type", "application/json" )
      self.send_header( "Content-length", str(len(msg)) )
      self.end_headers()
      self.wfile.write(msg)

    elif cmd == "pub":
      rwt = factory.get(topic)
      print "Pub[%s]" % topic

      if msgClasses.has_key(topic):
        msg = msgClasses[topic]()
      else:
        print "Unknow msg class[%s]" % topic

      if topic == "/move_base/activate":
        msg.header.frame_id = "/map"
      else:
        msg.covariance[6*0+0] = 0.5 * 0.5;
        msg.covariance[6*1+1] = 0.5 * 0.5;
        msg.covariance[6*3+3] = 3.14/12.0 * 3.14/12.0;

      msg.header.stamp = rospy.get_rostime()

      if qdict.has_key('x'):
        msg.pose.position.x = float(qdict['x'][0])
      else:
        msg.pose.position.x = 0.0
      if qdict.has_key('y'):
        msg.pose.position.y = float(qdict['y'][0])
      else:
        msg.pose.position.y = 0.0
      if qdict.has_key('z'):
        msg.pose.position.z = float(qdict['z'][0])
      else:
        msg.pose.position.z = 0.0

      if qdict.has_key('t'):
        q = bullet.Quaternion(float(qdict['t'][0]),0,0)
        msg.pose.orientation.x = float(q.x())
        msg.pose.orientation.y = float(q.y())
        msg.pose.orientation.z = float(q.z())
        msg.pose.orientation.w = float(q.w())
      else:
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

      rwt.publish(msg)

    elif cmd == "service":
      print "Service[%s]" % topic

      service_type = get_service_class(topic)
      service_class = roslib.scriptutil.get_service_class(service_type)

      rospy.wait_for_service(topic)
      service_proxy = rospy.ServiceProxy(topic, service_class)

      msg = ''

      if topic == '/static_map':
        msg = GetMapAsJPEG(service_proxy, qdict)
      else:
        print "Unable to handle service[%s]" % topic

      self.send_response(200)
      self.send_header( "Content-type", "application/json" )
      self.send_header( "Content-length", str(len(msg)) )
      self.end_headers()
      self.wfile.write(msg)

    elif cmd == "topics":
      m = roslib.scriptutil.get_master()
      code, _, topics = m.getPublishedTopics(CALLER_ID, '/')
      msg = '{"topics": ['
      for name, type in topics:
        msg += '{"name" : "' + name + '", "type" : "' + type + '"},'
      msg = msg.rstrip(',')
      msg += ']}'

      self.send_response(200)
      self.send_header( "Content-type", "application/json" )
      self.send_header( "Content-length", str(len(msg)) )
      self.end_headers()

      self.wfile.write(msg)
  #-----------------------------------------------------------------------------


  ##############################################################################
  # Handle a GET request
  def do_GET(self):
    (scm, netloc, path, params, query, fragment) = urlparse.urlparse( 
      self.path, 'http')

    # Query dictionary
    qdict = cgi.parse_qs(query)

    # If the request matches "/ros", then handle it ourselves
    if path.startswith("/ros"):
      self.handleROS(path, qdict)
    # Otherwise pass the request to the normal apache server
    else:
      #self.send_response(200);
      #self.send_header( "Content-type", "text/html" )
      #self.end_headers()
      #self.wfile.write("HELP")

      netloc = "localhost:80"
      scm = "http"
      soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      try:
        if self._connect_to(netloc, soc):
          #print "%s %s %s\r\n" % (
          #  self.command,
          #  urlparse.urlunparse(('', '', path, params, query, '')),
          #  self.request_version)
          soc.send("%s %s %s\r\n" % (
            self.command,
            urlparse.urlunparse(('', '', path, params, query, '')),
            self.request_version))
          self.headers['Connection'] = 'close'
          del self.headers['Proxy-Connection']
          for key_val in self.headers.items():
            soc.send("%s: %s\r\n" % key_val)
          soc.send("\r\n")
          self._read_write(soc)
      finally:
        soc.close();
        self.connection.close()
  #-----------------------------------------------------------------------------


  ##############################################################################
  def _read_write(self, soc, max_idling=20):
    iw = [self.connection, soc]
    ow = []
    date = ''
    count = 0
    while 1:
      count += 1
      (ins, _, exs) = select.select(iw, ow, iw, 3)
      if exs: break
      if ins:
        for i in ins:
          if i is soc:
            out = self.connection
          else:
            out = soc
          data = i.recv(8192)
          if data:
            out.send(data)
            
            count = 0
      else:
        pass
      if count == max_idling: break
  #-----------------------------------------------------------------------------
      

  ##############################################################################
  # Handle a POST
  do_POST = do_GET
  #-----------------------------------------------------------------------------

#*******************************************************************************

class MasterMonitor:
  def run(self):
    master = roslib.scriptutil.get_master()
    while 1:
      try:
        master.getSystemState('/rostopic')
      except:
        import traceback
        return
      time.sleep(.1)


class MyHTTPServer(threading.Thread, SocketServer.ThreadingMixIn, 
                   BaseHTTPServer.HTTPServer):
  def __init__(self, hostport, handler):
    threading.Thread.__init__(self)
    SocketServer.TCPServer.__init__(self, hostport, handler)
    
  def run(self):
    while 1:
      self.handle_request()



################################################################################
# Main
if __name__ == '__main__':

  print 'starting web server on port 8080'

  httpServer = MyHTTPServer( ('', 8080), Handler)
  httpServer.setDaemon(True)
  httpServer.start()

  try:
    rospy.init_node('webteleop', disable_signals=True)
    time.sleep(1)
    tfclient = TransformListener()

    mm = MasterMonitor()
    mm.run()
  finally:
    rospy.signal_shutdown('webteleop exiting')
    if httpServer: httpServer.server_close()

  #try:
  #  httpServer.serve_forever()
  #except KeyboardInterrupt:
  #  pass

  #httpServer.close()
  #
  rospy.signal_shutdown('webteleop exiting')
