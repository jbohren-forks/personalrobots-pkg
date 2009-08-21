#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

PKG_NAME = 'rosweb2'
import roslib; roslib.load_manifest(PKG_NAME)

import cStringIO
import os
import signal
import string
import sys
import threading
import time
import traceback
import BaseHTTPServer
import logging

import BaseHTTPServer, select, socket, SocketServer, urlparse
SocketServer.ThreadingMixIn.daemon_threads = True

import Cookie
import cgi

import roslib.scriptutil
import rospy
import rosservice
import rosjson

from rosservice import ROSServiceException
from roslib.message import ROSMessageException

class ROSWebException(Exception): pass

def splitTopic(topic):
  parts = topic.split("/")
  topic = "/" + parts[1]
  subtopic = None
  if len(parts) == 3: 
    subtopic = parts[2]
  return topic, subtopic


## Factory for ROSWebTopic instances
class RWTFactory(object):
    def __init__(self):
        self.map = {}
        self.lock = threading.Lock()

        self.counter = 1
        self.cond = threading.Condition()

    def close(self):
      try:
        self.lock.acquire()
        for topic in self.map.keys():
          rwt = self.map[topic]
          rwt.factory = None
          rwt = None
          del self.map[topic]
      finally:
        self.lock.release()

    def unsubscribe(self, topic):
      try:
        self.lock.acquire()
        if topic in self.map:
          del self.map[topic]
      finally:
        self.lock.release()

    def subscribe(self, topic):
        maintopic, subtopic = splitTopic(topic)
        main_rwt = None
        try:
            self.lock.acquire()
            if topic in self.map:
                return self.map[topic]
            main_rwt = ROSWebTopic(maintopic, self)
            self.map[maintopic] = main_rwt
            rwt = main_rwt
            if subtopic:
              rwt = ROSWebSubTopic(topic, self, main_rwt)
              self.map[topic] = rwt
        finally:
            self.lock.release()
        return main_rwt

    def get(self, topic):
      return self.map.get(topic, None)

    def registerVirtualTopic(self, topic, thread):
      vt = VirtualTopic(topic, self, thread)
      try:
        self.lock.acquire()
        self.map[topic] = vt
        vt.init()
      finally:
        self.lock.release()


class ROSWebTopic(object):
    def __init__(self, topic, factory):
        self.topic = topic
        self.initialized = False
        self.sub = None
        self.factory = factory

        self.last_message = None
        self.messages = []
        self.subtopics = []

    def init(self):
        if self.initialized: return

        try:
            self.factory.cond.acquire()
            m = roslib.scriptutil.get_master()

            code, _, topics = m.getPublishedTopics('/'+PKG_NAME, '/')
            if code != 1:
              raise ROSWebException("unable to communicate with master")
            
            for t, topic_type in topics:
                if t == self.topic: break
            else:
                raise ROSWebException("%s is not a published topic"%self.topic)
            
            msg_class = roslib.scriptutil.get_message_class(topic_type)

            self.sub = rospy.Subscriber(self.topic, msg_class, self.topic_callback)
            self.initialized = True
        except ROSWebException:
            raise
        except Exception, e:
            # fatal for now
            rospy.signal_shutdown(str(e))
            raise
        finally:
            self.factory.cond.release()

    def close(self):
      self.factory = None
      for rwt, subtopic in self.subtopics:
        subtopic.close()

    def topic_callback(self, data):
        try:
            self.factory.cond.acquire()
            self.messages.append((self.factory.counter, data))
            self.factory.counter = self.factory.counter + 1
            self.messages = self.messages[-20:]

            for rwt, subtopic in self.subtopics:
              rwt.subtopic_callback(data)

            self.factory.cond.notifyAll()
        finally:
            self.factory.cond.release()

    def subscribeSubTopic(self, rwt, subtopic):
      self.subtopics.append((rwt, subtopic))
      

class ROSWebSubTopic(object):
    def __init__(self, topic, factory, main_rwt):
      self.factory = factory
      self.topic = topic

      self.messages = []
      self.maintopic, self.subtopic = splitTopic(topic)
      main_rwt.subscribeSubTopic(self, self.subtopic)

    def init(self):
      pass

    def subtopic_callback(self, data):
      ## no locks needed since this will be called from the main topic

      if self.maintopic == "/diagnostics":
        for _msg in data.status:
          if _msg.name == self.subtopic:
            self.messages.append((self.factory.counter, data))
            self.factory.counter = self.factory.counter + 1
            self.messages = self.messages[-20:]

    

class VirtualTopic(object):
  def __init__(self, topic, factory, thread):
    self.factory = factory
    self.topic = topic

    self.messages = []

    self.thread = thread
    self.initialized = False

  def init(self):
    if not self.initialized:
      self.initialized = True
      self.thread.setCallback(self.callback)
      self.thread.start()

  def callback(self, data):
    try:
      self.factory.cond.acquire()
      self.messages.append((self.factory.counter, data))
      self.factory.counter = self.factory.counter + 1
      self.messages = self.messages[-3:]

      self.factory.cond.notifyAll()
    finally:
      self.factory.cond.release()
    

class ROSWebHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def __init__(self, *args):
        BaseHTTPServer.BaseHTTPRequestHandler.__init__(self, *args)

    def subscribe(self, topic):
        rwt = self.server.factory.subscribe(topic)
        try:
          rwt.init()
        except ROSWebException, e:
          self.server.factory.unsubscribe(topic)
          #traceback.print_exc()
          return None
        return rwt

    def _do_GET_topic(self, topics, since, callback=None):
        rwttopics = []

#        if "/topics" not in topics:
#          topics.append("/topics")

        for topic in topics:
          rwt = self.server.factory.get(topic)
          if rwt:
            rwttopics.append((topic, rwt))
          else:
            rwt = self.subscribe(topic)
            if rwt: rwttopics.append((topic, rwt))

        if len(rwttopics) == 0:
          logging.warning("no valid topics")
          self.send_response(404)
          return
          
        messages = []
        lastSince = 0
        try:
            self.server.factory.cond.acquire()
            # timeout?

            if since > self.server.factory.counter:
              ## reset the since if it is invalid.
              logging.error("invalid since counter, reseting... %s" % (since, ))
              since = 0

            ## check for some old messages first
            lastSince = since
            for topic, rwt in rwttopics:
              if not rwt: continue
              for t,msg in rwt.messages:
                if t > since:
                  messages.append((topic, msg))
                  lastSince = max(t, lastSince)
                
            ## if no old messages, then wait for a new one.
            while not messages:
              self.server.factory.cond.wait()

              # get the last message under lock
              for topic, rwt in rwttopics:
                if not rwt: continue
                for t,msg in rwt.messages:
                  if t > lastSince:
                    messages.append((topic, msg))
                    lastSince = max(t, lastSince)
        finally:
            self.server.factory.cond.release()
            
        buf = cStringIO.StringIO()
        buf.write("{")
        buf.write('"since": %s,' % lastSince)
        buf.write('"msgs": [')
        _i = 0
        for (topic, msg) in messages:
          _i = _i + 1
          if _i > 1: buf.write(',')
          buf.write("{")
          buf.write('"topic": "%s",' % topic)
          buf.write('"msg": ')
          buf.write(rosjson.ros_message_to_json(msg))
          buf.write("}")
        buf.write(']')
        buf.write('}')
          
        buf = buf.getvalue()

        self.send_success(buf, callback)

    def _connect_to(self, netloc, soc):
        i = netloc.find(':')
        if i >= 0:
          host_port = netloc[:i], int(netloc[i+1:])
        else:
            host_port = netloc, 80
#        print "\t" "connect to %s:%d" % host_port
        try: soc.connect(host_port)
        except socket.error, arg:
          try: msg = arg[1]
          except: 
            msg = arg
            self.send_error(404, msg)
            return 0
        return 1

    def send_success(self, buf = "{}", callback = None):
      self.send_response(200)
      if callback:
        buf = callback + "(" + buf + ");"
        self.send_header('Content-Type', 'text/javascript')
      else:
        self.send_header('Content-Type', 'application/json')
      self.send_header('Content-Length', str(len(buf)))
      self.end_headers()
      
      self.wfile.write(buf)

    def handle_ROS(self, path, qdict):
      cstring = self.headers.get('Cookie', '')

      cookie = Cookie.BaseCookie()
      cookie.load(cstring)

      path_parts = path.split("/")

      username = ""

      cmd = path_parts[2]

      ## parse out the username from the cookie
      if cookie.has_key('MB_L1'):
        l1 = cookie['MB_L1'].value
        if l1.startswith("V1/"):
          code = l1[3:]
          parts = code.split(":", 1)
          username = parts[0]

      ## Make a service call
      ##   Args can be passed as an ordered list of parameters:
      ##     /ros/service/<service_name>?args=<arg1>&args=<arg2>&<...>
      ##   or as a dictionary of named parameters
      ##     /ros/service/<service_name>?json=<json-encoded dictionary of args>
      if cmd == "service":
        name = path_parts[3]
        service_class = rosservice.get_service_class_by_name("/" + name)
        request = service_class._request_class()

        callback = qdict.get("callback", [''])[0]

        args = qdict.get('args', None)
        if not args:
          args = qdict.get('json', ['{}'])[0]
          args = eval(args)
        try:
          roslib.message.fill_message_args(request, args)
        except ROSMessageException:
          raise ROSServiceException("Not enough arguments to call service.\n"+\
              "Args should be: [%s], but are actually [%s] " %
              (roslib.message.get_printable_message_args(request), args))

        logging.debug("service call: name=%s, args=%s" % (name, args))
        rospy.wait_for_service(name)
        service_proxy = rospy.ServiceProxy(name, service_class)
        msg = service_proxy(request)
        msg = rosjson.ros_message_to_json(msg)

        logging.debug("service call: name=%s, resp=%s" % (name, msg))
        self.send_success(msg, callback=callback)
        return

      if cmd == "receive":
        since = int(qdict.get('since', ['0'])[0])
        topics = qdict.get("topic", "")
        callback = qdict.get("callback", [''])[0]
        self._do_GET_topic(topics, since, callback)
        return
      
    def do_GET(self):
        (scm, netloc, path, params, query, fragment) = urlparse.urlparse(
            self.path, 'http')
        qdict = cgi.parse_qs(query)
        pdict = cgi.parse_qs(self.rfile._rbuf)
        qdict.update(pdict)

        self.log_request()

        if path.startswith("/ros"):
          self.handle_ROS(path, qdict)
          return

        netloc = "localhost:80"
        scm = "http"

        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
          if self._connect_to(netloc, soc):
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
            soc.close()
            self.connection.close()

    def _read_write(self, soc, max_idling=20):
        iw = (self.connection, soc)
        ow = ()
        count = 0

        while 1:
            count += 1
            (ins, _, exs) = select.select(iw, ow, iw, 3)
            if exs: break
            if ins:
              for i in ins:
                if i is soc:
                  out = self.connection
                elif i is self.connection:
                  out = soc
                else:
                  print "error"
                data = i.recv(8192)
                if data:
                  bytes_sent = 0
                  while bytes_sent != len(data):
                    n = out.send(data[bytes_sent:])
                    bytes_sent += n
                  count = 0
            else:
                pass
            if count == max_idling: break

    do_POST = do_GET

    def log_message(self, format, *args):
      self.server.accesslog_fp.write("%s - - [%s] %s\n" %
                              (self.address_string(),
                               self.log_date_time_string(),
                               format%args))
      self.server.accesslog_fp.flush()

      
class MasterMonitor:
  def run(self):
    global running
    master = roslib.scriptutil.get_master()
    while running:
      try:
        master.getSystemState('/rostopic')
      except:
        import traceback
        #traceback.print_exc()
        return
      time.sleep(1)

class ThreadingHTTPServer (threading.Thread,
                           SocketServer.ThreadingMixIn,
                           BaseHTTPServer.HTTPServer): 
  def __init__(self, hostport, handlerClass):
    threading.Thread.__init__(self)
    SocketServer.TCPServer.__init__(self, hostport, handlerClass)

    self.factory = RWTFactory()
    
    self.registerVirtualTopics()

    self.accesslog_fp = open("access_log", "a+")

  def registerVirtualTopics(self):
    import users
    ut = users.UsersThread()
    ut.setDaemon(True)
    self.factory.registerVirtualTopic("/users", ut)

    import topics
    ut = topics.TopicThread()
    ut.setDaemon(True)
    self.factory.registerVirtualTopic("/topics", ut)

  def run(self):
    while 1:
      self.handle_request()

def rosweb_start():
    web_server = None
    mm = None

    web_server = ThreadingHTTPServer(('', 8080), ROSWebHandler)
    web_server.setDaemon(True)
    web_server.start()
    logging.info("starting Web server")

    try:
      rospy.init_node(PKG_NAME, disable_signals=True)

      mm = MasterMonitor()
      mm.run()

    finally:
      rospy.signal_shutdown(PKG_NAME + ' exiting')
      if web_server: web_server.server_close()

error_log_fp = None

def signal_handler(signal, frame):
  global running
  running = False

def main():
  global running, error_log
  running = True
  logging.basicConfig(filename="error_log", level=logging.DEBUG)

  signal.signal(signal.SIGINT, signal_handler)

  while running:
    logging.info("starting")
    try:
      rosweb_start()
    except:
      import traceback
      fp = open("error_log", "a+")
      traceback.print_exc(file=fp)
      fp.close()
    time.sleep(1)

  if error_log_fp: error_log_fp.close()

if __name__ == '__main__':
  main()
