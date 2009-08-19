#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

PKG = 'taskman' # this package name
NAME = 'taskman'

import roslib; roslib.load_manifest(PKG) 

from taskman.srv import *
from taskman.msg import *
import rospy 

import roslib.names
import roslib.network 

from roslaunch.config import ROSLaunchConfig
from roslaunch.launch import ROSLaunchRunner
from roslaunch.pmon import pmon_shutdown as _pmon_shutdown
from roslaunch.xmlloader import *

import subprocess

from taskman import app

def getPackagePath(pkg):
  pkgpath = subprocess.Popen(["rospack", "find", pkg], stdout=subprocess.PIPE).communicate()[0].strip()
  return pkgpath

class TaskGroup:
  def __init__(self, manager):
    self.task = TaskStatus(None, None, None, None, None)
    self.app = None
    self.runner = None
    self.childGroups = []
    self.manager = manager

  def __del__(self):
    if self.manager: self.manager = None
    if self.childGroups: self.childGroups = None

  def launch(self):
    config = ROSLaunchConfig()
    loader = XmlLoader()
    path = getPackagePath(self.app.package)
    print "pkgpath", path
    print "launchfile", self.app.launch_file
    #os.chdir(pkgpath)
    fn = os.path.join(path, self.app.launch_file)
    try:
      loader.load(fn, config)
    except:
      self.task.status = "error"
      self.manager.pub.publish(self.task)
      return

    self.runner = ROSLaunchRunner(rospy.get_param("/run_id"), config, is_core=False)
    self.runner.pm.add_process_listener(self)
    self.runner.launch() 

  def stop(self):
    if not self.runner: return
    self.runner.stop()
    self.runner = None

  def process_died(self, process_name, exit_code):
    #print "process_died", process_name, exit_code
    #print self.runner.pm.procs
    if len(self.runner.pm.procs) == 1:
      self.runner = None
      #print "ALL DONE!"
      self.manager._stopTask(self)
      

  def __repr__(self):
    return "<TaskGroup %s %s %s>" % (self.app.provides, self.app.taskid, len(self.childGroups))



class TaskManager:
  def __init__(self):
    self.pub = rospy.Publisher("TaskStatus", TaskStatus, self)
    self._taskGroups = {}
    self._apps = {}

  def start_task(self, req):
    app = app.App(req.taskid)
    pgroup = None
    group = self._taskGroups.get(app.provides, None)
    if group:
      if group.task.taskid == req.taskid:
        print "already running"
        return StartTaskResponse("already running")

      self._stopTask(group)

    if app.depends and app.depends.strip():
      print "depends", app.depends
      pgroup = self._taskGroups.get(app.depends, None)
      if not pgroup:
        print "no parent task group %s running." % str(app.depends)
        return StartTaskResponse("no parent task group %s running." % str(app.depends))

    self._apps[req.taskid] = app
    group = TaskGroup(self)
    group.app = app
    if app.provides:
      self._taskGroups[app.provides] = group

    if pgroup:
      pgroup.childGroups.append(group)

    print "startTask [%s, %s, %s]" % (req.taskid, app.name, req.username)
    group.task.taskid = req.taskid
    group.task.username = req.username
    group.task.started = rospy.get_rostime()
    group.task.status = "starting"

    self.pub.publish(group.task)

    group.launch()

    group.task.status = "running"
    self.pub.publish(group.task)

    return StartTaskResponse("done")

  def showstate(self):
    print "_" * 40
    for provides, group in self._taskGroups.items():
      print provides, group.childGroups

  def _stopTask(self, group):
    print "_stopTask", group.app.provides

    group.task.status = "stopping"
    self.pub.publish(group.task)

    print "childGroups", group, group.childGroups
    for cgroup in group.childGroups[:]:
      self._stopTask(cgroup)

    group.stop()

    group.task.status = "stopped"
    self.pub.publish(group.task)

    if group.app.depends:
      pgroup = self._taskGroups.get(group.app.depends.strip(), None)
      if pgroup:
        pgroup.childGroups.remove(group)

    if group.app.provides:
      print "removing", group.app.provides
      del self._taskGroups[group.app.provides]

  def stop_task(self, req):
    app = self._apps[req.taskid]

    group = self._taskGroups.get(app.provides, None)
    if not group: 
      msg = "no such group: " + str(app.provides)
      return StopTaskResponse(msg)

    self._stopTask(group)

    del self._apps[req.taskid]

    return StopTaskResponse("done")

  def status_update(self, req):
    for provides, group in self._taskGroups.items():
      self.pub.publish(group.task)
    return StatusUpdateResponse("done")


  def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    pass

  def peer_unsubscribe(self, topic_name, numPeers):
    pass


def server():
    rospy.init_node(NAME)

    tm = TaskManager()
    s1 = rospy.Service('start_task', StartTask, tm.start_task)
    s2 = rospy.Service('stop_task', StopTask, tm.stop_task)
    s3 = rospy.Service('status_update', StatusUpdate, tm.status_update)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    server()
