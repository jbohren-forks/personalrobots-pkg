#! /usr/bin/env python

import os, sys

rosroot = '/u/hassan/pr2/ros'
sys.path.insert(0, os.path.join(rosroot, "core/roslib/src"))

os.environ['ROS_ROOT'] = rosroot
os.environ['ROS_PACKAGE_PATH'] =  '/u/hassan/pr2/ros-pkg'
os.environ['ROS_MASTER_URI'] =  'http://localhost:11311/'
os.environ['PATH'] = os.path.join(rosroot, "bin") + ":" + os.environ.get('PATH', "")
os.environ['HOME'] = '/tmp'

PKG = 'webui' # this package name
import roslib; roslib.load_manifest(PKG) 

from pyclearsilver import cgistarter

from webui import config

path,f = os.path.split(__file__)
#sys.stderr.write("path: %s\n"  % path)
os.chdir(path)

cgistarter.setConfig(config)
handler = cgistarter.handler

