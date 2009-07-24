#!/usr/bin/env python

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

from nav_srvs.srv import *
from nav_msgs.msg import *
from std_msgs.msg import *
from robot_msgs.msg import *

CALLER_ID = '/webteleop'
index = 0

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

factory = RWTFactory()

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

      msg_class = roslib.scriptutil.get_message_class(self.topic_type)

      if self.pubsub == "subscriber":
        self.sub = rospy.Subscriber(self.topic, msg_class, self.topic_callback)
      else:
        self.pub = rospy.Publisher(self.topic, msg_class)

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
      else:
        msg = rosjson.ros_message_to_json(self.last_message)

    finally:
      self.cond.release()

    return msg
  #-----------------------------------------------------------------------------
#*******************************************************************************


    
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

      # Wait one second. This makes ros happy. Should be removed when
      # latching topics (ROS 0.7??) is implemented
      time.sleep(1)

      self.send_response(200)
      self.send_header( "Content-type", "text/html" )
      self.end_headers()
      self.wfile.write("announced")

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

      if topic == "/move_base/activate":
        msg = robot_msgs.msg.PoseStamped()
      else:
        msg = robot_msgs.msg.PoseWithCovariance()

      msg.header.stamp = rospy.get_rostime()
      msg.header.frame_id = "/map"

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


      msg.pose.orientation.x = 0;
      msg.pose.orientation.y = 0;
      msg.pose.orientation.z = 0;
      msg.pose.orientation.w = 1;

      print msg
      rwt.publish(msg)

    elif cmd == "service":
      print "Service[%s]" % topic

      rospy.wait_for_service('/static_map')

      try:
        staticMap = rospy.ServiceProxy('/static_map', StaticMap)
        map = staticMap()
      except rospy.ServiceException, e:
        print "Service call exception: %s" % e

      mapW = map.map.info.width
      mapH = map.map.info.height
      data = map.map.data

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

      self.send_response(200)
      self.send_header( "Content-type", "application/json" )
      self.send_header( "Content-length", str(len(msg)) )
      self.end_headers()
      self.wfile.write(msg)

    elif cmd == "topics":
      print "Get topics"

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

    print "do_GET"
    print "[%s] [%s] [%s] [%s] [%s] [%s]" % (scm, netloc, path, params, query, fragment)

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
          print "Send:"
          print "%s %s %s\r\n" % (
            self.command,
            urlparse.urlunparse(('', '', path, params, query, '')),
            self.request_version)
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
            #dataIndex = data.find("Date: ")
            #index = data.find("GMT")
            #if dataIndex > 0 and index > 0:
            #  time = data[dataIndex+6:index+3]
            #  part = data[:index+3]
            #  part += '\n\rLast-modified: %s' % time
            #  part += data[index+4:]
            #  print "Part[%s]" % part
            #  #print "Data[%s]" %data
            #  out.send(part)
            #else:
            print data
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
      time.sleep(1)

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

  print 'starting web server'

  #httpServer = BaseHTTPServer.HTTPServer( ('', 8080), Handler)
  httpServer = MyHTTPServer( ('', 8080), Handler)
  httpServer.setDaemon(True)
  httpServer.start()

  try:
    rospy.init_node('webteleop', disable_signals=True)
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
