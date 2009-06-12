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
#  * Neither the name of the Willow Garage nor the names of its
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

import roslib
roslib.load_manifest('vslam_demo')
import rospy
import tf.transformations as transformations

import sys
import math
import time

########################################################################

from visual_odometry.visualodometer import VisualOdometer, Pose, from_xyz_euler
from stereo_utils.stereo import ComputedDenseStereoFrame, SparseStereoFrame
from stereo_utils.descriptor_schemes import DescriptorSchemeCalonder, DescriptorSchemeSAD
from stereo_utils.feature_detectors import FeatureDetectorFast
from stereo_utils.camera import VidereCamera

from skeleton import Skeleton

import cv

import dcam

class RealSource:

  def __init__(self):
    self.dc = dcam.dcam()

  def cam(self):
    return VidereCamera(self.dc.retParameters())

  def getImage(self):

    class Im:
      def __init__(self, size, i):
        self.size = size
        self.data = i
      def tostring(self):
        return self.data

    w,h,l,r = self.dc.getImage()
    sz = (w, h)
    li = Im(sz, l)
    ri = Im(sz, r)

    return w,h,li,ri

from stereo_utils.reader import reader

class FakeSource:

  def __init__(self, source):
    self.r = reader(source)

  def cam(self):
    (cam, l, r) = self.r.next()
    return cam

  def getImage(self):
    (cam, l, r) = self.r.next()
    (w, h) = l.size
    return (w, h, l, r)

class Demo:
  def __init__(self, source):
    cv.NamedWindow("Camera")
    cv.MoveWindow("Camera", 0, 500)
    self.cvim = None

    self.fd = FeatureDetectorFast(300)
    self.ds = DescriptorSchemeCalonder()
    self.stereo_cam = source.cam()
    self.vo = VisualOdometer(self.stereo_cam, scavenge = False, sba=None, inlier_error_threshold = 3.0)
    self.skel = Skeleton(self.stereo_cam, descriptor_scheme = self.ds, optimize_after_addition = False)
    self.connected = False
    self.source = source

  def handleFrame(self):
    (w,h,li,ri) = self.source.getImage()

    f = SparseStereoFrame(li, ri, feature_detector = self.fd, descriptor_scheme = self.ds)
    self.vo.handle_frame(f)
    self.skel.add(self.vo.keyframe, self.connected)
    self.connected = True

    if 1:
      # Run the OpenCV preview
      if not self.cvim:
        self.cvim = cv.CreateImage((w,h/2), cv.IPL_DEPTH_8U, 1)

      im = cv.CreateImage((w,h), cv.IPL_DEPTH_8U, 1)
      im2 = cv.CreateImage((w/2,h/2), cv.IPL_DEPTH_8U, 1)
      for i,src in enumerate([li, ri]):
        cv.SetData(im, str(src.tostring()), w)
        cv.Resize(im, im2)
        cv.SetImageROI(self.cvim, (i * w / 2, 0, w / 2, h / 2))
        cv.Copy(im2, self.cvim)
      cv.ResetImageROI(self.cvim)

      cv.ShowImage("Camera", self.cvim)
      cv.WaitKey(10)

  def pose(self):
    return self.vo.pose

if len(sys.argv) == 1:
  demo = Demo(RealSource())
else:
  demo = Demo(FakeSource(sys.argv[1]))

while False:
  demo.handleFrame()
  print demo.skel.nodes
  for id in demo.skel.nodes:
    print demo.skel.newpose(id)

########################################################################

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

# Some api in the chain is translating the keystrokes to this octal string
# so instead of saying: ESCAPE = 27, we use the following.
ESCAPE = '\033'

# Number of the glut window.
window = 0

import numpy
ball = transformations.Arcball(initial=numpy.identity(4))
ball.place([320, 240], 320)

def InitGL(Width, Height):				# We call this right after our OpenGL window is created.
  glClearColor(0.0, 0.0, 0.0, 0.0)	                # This Will Clear The Background Color To Black
  glClearDepth(1.0)					# Enables Clearing Of The Depth Buffer
  glDepthFunc(GL_LESS)				        # The Type Of Depth Test To Do
  glEnable(GL_DEPTH_TEST)				# Enables Depth Testing
  glShadeModel(GL_SMOOTH)				# Enables Smooth Color Shading
  
  glMatrixMode(GL_PROJECTION)
  glLoadIdentity()					# Reset The Projection Matrix
                                                        # Calculate The Aspect Ratio Of The Window
  gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)

  glMatrixMode(GL_MODELVIEW)

# The function called when our window is resized (which shouldn't happen if you enable fullscreen, below)
def ReSizeGLScene(Width, Height):
  if Height == 0:					# Prevent A Divide By Zero If The Window Is Too Small 
    Height = 1

  glViewport(0, 0, Width, Height)		        # Reset The Current Viewport And Perspective Transformation
  glMatrixMode(GL_PROJECTION)
  glLoadIdentity()
  gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
  #glMatrixMode(GL_MODELVIEW)
  #gluLookAt(0.0, 0.0, 3.0,   0.0, 0.0, 0.0,   0.0, 1.0, 0.0);      

def cube():
  glBegin(GL_LINES)
  for a in [ -1, 1 ]:
    for b in [ -1, 1 ]:
      glVertex3f(a, b, -1)
      glVertex3f(a, b, 1)
      glVertex3f(a, -1, b)
      glVertex3f(a, 1, b)
      glVertex3f(-1, a, b)
      glVertex3f(1, a, b)
  glEnd()

def camera():
  glPushMatrix()
  glScalef(.1, .1, .1)
  cube()

  glBegin(GL_LINES)
  glVertex3f(0,0,0)
  glVertex3f(0,0,2)
  glEnd()

  glBegin(GL_TRIANGLE_FAN)
  glVertex3f(0,0,2)
  for i in range(31):
    th = i * 2 * math.pi / 30
    glVertex3f(math.sin(th) * 0.25, math.cos(th) * .25, 1.5)
  glEnd()

  glPopMatrix()

def draw_view():
  
  mod = {
    'a' : (0, 0, 0),
    'b' : (-.1, -.1, .1),
    'c' : (-.1, .1, .1),
    'd' : (.1, .1, .1),
    'e' : (.1, -.1, .1) }

  glColor3f(.8, .7, .6)
  glBegin(GL_LINES)
  for p0,p1 in [ ('a','b'), ('a','c'), ('a','d'), ('a','e'), ('b','c'), ('c','d'), ('d','e'), ('e','b') ]:
    glVertex3f(*mod[p0])
    glVertex3f(*mod[p1])
  glEnd()

def poseMultMatrix(pose):
  M = numpy.dot(numpy.array([[-1, 0, 0, 0],
                             [ 0, -1, 0, 0],
                             [ 0, 0, 1, 0],
                             [ 0, 0, 0, 1]]), pose.M)
  glMultMatrixf(sum(M.T.tolist(), []))

def DrawGLScene():
  glClearColor(0, 0, 0, 1)
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

  demo.handleFrame()
  glColor3f(1, 1, 1)
  cube()

  glColor3f(.3, .3, .3)
  glBegin(GL_QUADS)
  for x,z in [ (-1,-1), (-1,1), (1,1), (1,-1) ]:
    glVertex3f(x, -1, z)
  glEnd()

  glMatrixMode(GL_MODELVIEW)

  # Plot the skeleton

  glColor3f(.3, .3, 1)
  glLineWidth(3)
  glBegin(GL_LINES)
  for i0,i1 in demo.skel.edges:
    (x,y,z) = demo.skel.newpose(i0).xform(0,0,0)
    glVertex3f(-x,-y,z)
    (x,y,z) = demo.skel.newpose(i1).xform(0,0,0)
    glVertex3f(-x,-y,z)
  glEnd()
  glLineWidth(1)

  for id in demo.skel.nodes:
    glPushMatrix()
    poseMultMatrix(demo.skel.newpose(id))
    draw_view()
    glPopMatrix()

  if 0:
    glTranslatef(0, 0.5, 0)
  else:
    poseMultMatrix(demo.pose())

  glColor3f(1, 1, 1)
  camera()

  glutSwapBuffers()

class MouseLook:

  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z
    self.th = 0
    self.phi = 0
    self.a_fwd = 0
    self.v_fwd = 0
    self.a_rgt = 0
    self.v_rgt = 0
    self.a_clm = 0
    self.v_clm = 0
    self.last = time.time()

  def matrix(self):
    R = transformations.euler_matrix(self.phi, self.th, 0)
    T = transformations.translation_matrix([self.x, self.y, self.z])
    M = numpy.dot(T, R)
    return M

  def turn_u(self, d):
    self.phi -= d

  def turn_r(self, d):
    self.th -= d

  def translate(self, x, y, z):
    P = self.matrix()
    T = transformations.translation_matrix([x, y, z])
    M = numpy.dot(P, T)
    #print "P:\n", P
    #print "T:\n", T
    #print "M:\n", M
    self.x = M[0,3]
    self.y = M[1,3]
    self.z = M[2,3]

  def frob(self):
    t = time.time()
    delta = t - self.last
    self.last = t

    self.v_fwd += delta * self.a_fwd
    if self.a_fwd:
      self.v_fwd *= 0.99
    else:
      self.v_fwd *= 0.7

    self.v_rgt += delta * self.a_rgt
    if self.a_rgt:
      self.v_rgt *= 0.99
    else:
      self.v_rgt *= 0.7

    self.v_clm += delta * self.a_clm
    if self.a_clm:
      self.v_clm *= 0.99
    else:
      self.v_clm *= 0.7

    self.translate(self.v_rgt * delta, self.v_clm * delta, -self.v_fwd * delta)

  def unkey(self, k):
    if k == 'w':
      self.a_fwd = 0
    if k == 's':
      self.a_fwd = 0
    if k in [ 'a', 'd' ]:
      self.a_rgt = 0
    if k in [ 'c', ' ' ]:
      self.a_clm = 0
    
  def key(self, k):
    th_inc = 0.05
    if k == 'w':
      self.a_fwd = 2
    elif k == 's':
      self.a_fwd = -2
    elif k == 'a':
      self.a_rgt = -2
    elif k == 'd':
      self.a_rgt = 2
    elif k == ' ':
      self.a_clm = 2
    elif k == 'c':
      self.a_clm = -2
    elif k == GLUT_KEY_LEFT:
      self.turn_r(-th_inc)
    elif k == GLUT_KEY_RIGHT:
      self.turn_r(th_inc)

  def passive(self, x, y):
    if x != 320 or y != 240:
      self.turn_r((x - 320) * 0.001)
      self.turn_u((y - 240) * 0.001)
      glutWarpPointer(320,240)

ml = MouseLook(0,0,4)

def keyPressed(*args):
  global window
  if args[0] == ESCAPE:
    sys.exit()
  else:
    ml.key(args[0])

def keyUnpressed(*args):
  global window
  if args[0] == ESCAPE:
    sys.exit()
  else:
    ml.unkey(args[0])

def specialKeyPressed(key, x, y):
  ml.key(key)

def updateMV():
  glMatrixMode(GL_MODELVIEW)
  if 0:
    glLoadIdentity()
    glTranslatef(0, 0, -4)
    glMultMatrixf(ml.matrix().T)
  else:
    M = ml.matrix()
    M = numpy.linalg.inv(M)
    L = (sum(M.T.tolist(), []))
    glLoadMatrixf(L)
  #print "GL has:\n", glGetFloatv(GL_MODELVIEW_MATRIX)

if 0:
  def mouseEvent(button, state, x, y):
    if button == 0 and state == 0:
      ball.down([640 - x, 480 - y])

  def updateMV():
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glTranslatef(0, 0, -4)
    glMultMatrixf(ball.matrix())

  def motionEvent(x, y):
    ball.drag([640 - x, 480 - y])
    updateMV()

def every():
  ml.frob()
  updateMV()
  DrawGLScene()

def main():
  global window
  glutInit(sys.argv)

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)

  glutInitWindowSize(640, 480)

  glutInitWindowPosition(0, 0)

  window = glutCreateWindow("VSLAM demo")

  glutDisplayFunc(every)

  # Uncomment this line to get full screen.
  #glutFullScreen()

  glutIdleFunc(every)

  glutReshapeFunc(ReSizeGLScene)

  glutKeyboardFunc(keyPressed)
  glutKeyboardUpFunc(keyUnpressed)
  glutSpecialFunc(specialKeyPressed)
  glutIgnoreKeyRepeat(True)
  if 0:
    glutMouseFunc(mouseEvent)
    glutMotionFunc(motionEvent)
  glutPassiveMotionFunc(ml.passive)

  glutWarpPointer(320,240)
  glutSetCursor(GLUT_CURSOR_NONE)
  InitGL(640, 480)
  updateMV()

  glutMainLoop()

print "Hit ESC key to quit."
main()
