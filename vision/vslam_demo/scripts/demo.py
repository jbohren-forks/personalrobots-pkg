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
from math import *

########################################################################

from visual_odometry.visualodometer import VisualOdometer, Pose, from_xyz_euler
from stereo_utils.stereo import ComputedDenseStereoFrame, SparseStereoFrame
from stereo_utils.descriptor_schemes import DescriptorSchemeCalonder, DescriptorSchemeSAD
from stereo_utils.feature_detectors import FeatureDetectorFast, FeatureDetectorStar
from stereo_utils.camera import VidereCamera

from skeleton import Skeleton

import cv

class RealSource:

  def __init__(self):
    import dcam
    self.dc = dcam.dcam()

  def cam(self):
    paramstr = self.dc.retParameters()
    return VidereCamera(paramstr)

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
    self.cold = True

  def cam(self):
    self.started = time.time()
    (cam, l, r) = self.r.next()
    return cam

  def getImage(self):
    try:
      (cam, l, r) = self.r.next()
    except StopIteration:
      if self.cold:
        print "Took %f" % (time.time() - self.started)
        self.cold = False
      l,r = self.prev

    self.prev = (l,r)
    (w, h) = l.size
    return (w, h, l, r)

class Demo:
  def __init__(self, source):
    self.cvim = None

    # These variables are internal:

    self.stereo_cam = source.cam()
    print "Camera is", self.stereo_cam
    self.connected = False
    self.snail_trail = []
    self.source = source
    self.inlier_history = []
    self.label = 0
    self.skel_dist_thresh = 0.1;        # in meters

    # These variables can be tweaked:

    self.fd = FeatureDetectorFast(300)
#    self.fd = FeatureDetectorStar(500)
    self.ds = DescriptorSchemeCalonder()
    self.camera_preview = False
    self.vo = VisualOdometer(self.stereo_cam,
                             scavenge = False,
                             position_keypoint_thresh = 0.1,
                             angle_keypoint_thresh = 3*pi/180,
                             sba=None,
                             num_ransac_iters=500,
                             inlier_error_threshold = 3.0)
    self.skel = Skeleton(self.stereo_cam,
                         link_thresh = 100,
                         descriptor_scheme = self.ds,
                         optimize_after_addition = False)
    self.skel.node_vdist = 0
    self.running = -1                   # run forever

    if self.camera_preview:
      cv.NamedWindow("Camera")
      cv.MoveWindow("Camera", 0, 700)

  def handleFrame(self):
    r = False

    (w,h,li,ri) = self.source.getImage()

    self.f = SparseStereoFrame(li, ri,
                               disparity_range = 96,
                               feature_detector = self.fd,
                               descriptor_scheme = self.ds)

    self.vo.handle_frame(self.f)
    print self.vo.inl
    self.inlier_history = self.inlier_history[-20:] + [self.vo.inl]
    # If the VO inlier count falls below 20, we're not well-connected for this link
    if self.vo.inl < 20:
      self.connected = False
      self.f.pose = Pose()
      self.snail_trail = []

    # Add a frame to graph if:
    #   Frame zero, or
    #   There have been N successful preceding frames, and either
    #   Not connected,
    #   or if connected, the distance from the youngest graph frame to this frame is > thresh

    add_to_graph = None
    if self.vo.keyframe.id == 0 and len(self.skel.nodes) == 0:
      add_to_graph = "Zero keyframe"
    elif min(self.inlier_history) > 20:
      if not self.connected:
        add_to_graph = "Unconnected - but now recovering good poses"
        # This is a new graph, so generate a new label
        self.label += 1
      else:
        relpose = ~self.last_added_pose * self.f.pose
        if relpose.distance() > self.skel_dist_thresh:
          add_to_graph = "Connected and distance thresh passed"

    if add_to_graph:
      self.skel.setlabel(self.label)
      self.skel.add(self.vo.keyframe, self.connected)
      print "ADD %d %s" % (self.f.id, add_to_graph)
      self.connected = True
      self.last_added_pose = self.f.pose
      self.snail_trail = []
#      self.ioptimize(10)


    self.snail_trail.append(self.f.pose)

    # Render the camera view using OpenCV, then load it into an OpenGL texture
    if True:
      if not self.cvim:
        self.cvim = cv.CreateImage((w,h/2), cv.IPL_DEPTH_8U, 3)

      im = cv.CreateImage((w,h), cv.IPL_DEPTH_8U, 1)
      im2 = cv.CreateImage((w/2,h/2), cv.IPL_DEPTH_8U, 1)
      for i,src in enumerate([li, ri]):
        cv.SetData(im, str(src.tostring()), w)
        cv.Resize(im, im2)
        cv.SetImageROI(self.cvim, (i * w / 2, 0, w / 2, h / 2))
        cv.CvtColor(im2, self.cvim, cv.CV_GRAY2BGR)

      cv.ResetImageROI(self.cvim)

      def annotate(cvim):
        green = cv.RGB(0,255,0)
        red = cv.RGB(0,0,255)
        def half(p):
          return (p[0] / 2, p[1] / 2)
        for (x,y,d) in self.f.features():
          cv.Circle(cvim, half((x, y)), 2, green)
          d = int(d)
          cv.Line(cvim, ((w+x-d)/2, y/2 - 2), ((w+x-d)/2, y/2 + 2), green)
        a_features = self.vo.keyframe.features()
        b_features = self.f.features()
        inliers = set([ (b,a) for (a,b) in self.vo.pe.inl])
        outliers = set(self.vo.pairs) - inliers
        for (a,b) in inliers:
          cv.Line(cvim, half(b_features[b]), half(a_features[a]), green)
        for (a,b) in outliers:
          cv.Line(cvim, half(b_features[b]), half(a_features[a]), red)

      annotate(self.cvim)
      glBindTexture(GL_TEXTURE_2D, demo.textures[2])
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, 640, 240, 0, GL_RGB, GL_UNSIGNED_BYTE, self.cvim.tostring())

    if self.camera_preview:
      cv.ShowImage("Camera", self.cvim)
      if cv.WaitKey(10) == 27:
        r = True

    return r

  def anchor(self):
    """ Return the current pose of the most recently added skeleton node """
    id = max(self.skel.nodes)
    return self.skel.newpose(id)

  def optimize(self):
    self.skel.optimize(1000)

  def ioptimize(self,iters = 30):
    self.skel.ioptimize(iters)

  def report(self):
    print
    print
    print
    print "-" * 80
    print self.skel.edges
    print "-" * 80

  def pose(self):
    """ Return the current camera pose in the skeleton frame """
    if len(self.skel.nodes) == 0:
      return self.vo.pose
    else:
      return self.anchor() * (~self.snail_trail[0] * self.snail_trail[-1])

  def map_size(self):
    xyz = [demo.skel.newpose(i).xform(0,0,0) for i in demo.skel.nodes]
    xrange = max([x for (x,y,z) in xyz]) - min([x for (x,y,z) in xyz])
    zrange = max([z for (x,y,z) in xyz]) - min([z for (x,y,z) in xyz])
    return max(xrange, zrange)

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

BOTTOM_2D_HEIGHT = 240
screensize = ( 640, 480 + BOTTOM_2D_HEIGHT)

def InitGL(Width, Height):				# We call this right after our OpenGL window is created.
  glClearColor(0.0, 0.0, 0.0, 0.0)	                # This Will Clear The Background Color To Black
  glClearDepth(1.0)					# Enables Clearing Of The Depth Buffer
  glDepthFunc(GL_LESS)				        # The Type Of Depth Test To Do
  glEnable(GL_DEPTH_TEST)				# Enables Depth Testing
  # glDisable(GL_DEPTH_TEST)				# Enables Depth Testing
  glShadeModel(GL_SMOOTH)				# Enables Smooth Color Shading

  glFogf(GL_FOG_START, 10.0)
  glFogf(GL_FOG_END, 700.0)
  glFogi(GL_FOG_MODE, GL_LINEAR)
  glEnable(GL_FOG)

                                                        # Enable AA lines
  glEnable(GL_LINE_SMOOTH)
  glEnable(GL_BLEND)
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
  
  glMatrixMode(GL_PROJECTION)
  glLoadIdentity()					# Reset The Projection Matrix
                                                        # Calculate The Aspect Ratio Of The Window
  gluPerspective(45.0, float(Width)/float(Height-BOTTOM_2D_HEIGHT), 0.1, 1000.0)

  glMatrixMode(GL_MODELVIEW)

# The function called when our window is resized (which shouldn't happen if you enable fullscreen, below)
def ReSizeGLScene(Width, Height):
  if Height == 0:					# Prevent A Divide By Zero If The Window Is Too Small 
    Height = 1

  glMatrixMode(GL_PROJECTION)
  glLoadIdentity()
  # gluPerspective(45.0, float(Width)/float(Height-BOTTOM_2D_HEIGHT), 0.1, 1000.0)
  gluPerspective(45.0, float(Width)/float(Height), 0.1, 1000.0)
  print '----> aspect ----', float(Width)/float(Height-BOTTOM_2D_HEIGHT)
  global screensize
  screensize = (Width, Height)

def draw_cube():
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

def draw_camera():
  glPushMatrix()
  glScalef(.1, .1, .1)
  draw_cube()

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


# Display list names
LIST_EVERYTHING = 1   # The whole scene, used when paused
LIST_FURNITURE  = 2   # Fixed background stuff

def square(x0, y0, x1, y1):
  glBegin(GL_QUADS)
  glTexCoord2f(0,1)
  glVertex3f(x0, y0,0)
  glTexCoord2f(1,1)
  glVertex3f(x1, y0,0)
  glTexCoord2f(1,0)
  glVertex3f(x1, y1,0)
  glTexCoord2f(0,0)
  glVertex3f(x0, y1,0)
  glEnd()

def draw_logo():
  glMatrixMode(GL_MODELVIEW)
  glPushMatrix()
  glLoadIdentity()
  glMatrixMode(GL_PROJECTION)
  glPushMatrix()
  glLoadIdentity()

  glEnable(GL_TEXTURE_2D)
  glBindTexture(GL_TEXTURE_2D, demo.textures[ml.mode])

  w = 2 * (32. / screensize[0])
  h = 2 * (32. / (screensize[1] - BOTTOM_2D_HEIGHT))
  glColor3f(1.0, 1.0, 1.0)
  square(-1, -1, -1+w, -1+h)

  glDisable(GL_TEXTURE_2D)

  glMatrixMode(GL_PROJECTION)
  glPopMatrix()
  glMatrixMode(GL_MODELVIEW)
  glPopMatrix()

def draw_image():
  glMatrixMode(GL_MODELVIEW)
  glPushMatrix()
  glLoadIdentity()
  glMatrixMode(GL_PROJECTION)
  glPushMatrix()
  glLoadIdentity()

  glColor3f(1,1,1)
  glEnable(GL_TEXTURE_2D)

  w = 3 * (32. / screensize[0])
  h = 2 * (32. / screensize[1])

  glBindTexture(GL_TEXTURE_2D, demo.textures[2])
  square(-1, -1, 1, 1)

  glDisable(GL_TEXTURE_2D)

  glMatrixMode(GL_PROJECTION)
  glPopMatrix()
  glMatrixMode(GL_MODELVIEW)
  glPopMatrix()

class GLDemo(Demo):

  def __init__(*args):
    self = args[0]
    self.dl_ready = False    # display list ready
    Demo.__init__(*args)

  def redraw(self):
    w,h = screensize
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(w)/float(h-BOTTOM_2D_HEIGHT), 0.1, 1000.0)
    glMatrixMode(GL_MODELVIEW)
    glViewport(0, BOTTOM_2D_HEIGHT, w, h-BOTTOM_2D_HEIGHT)
    self.redraw_3d()
    glViewport(0, 0, w, BOTTOM_2D_HEIGHT)
    draw_image()
    
    glutSwapBuffers()
    
  def redraw_3d(self):
    glClearColor(0, 0, 0, 1)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    if demo.running != 0:
      if demo.handleFrame():
        sys.exit()
      demo.running -= 1

    glMatrixMode(GL_MODELVIEW)

    if demo.running == 0:
      if not self.dl_ready:
        glNewList(LIST_EVERYTHING, GL_COMPILE_AND_EXECUTE)
      else:
        glCallList(LIST_EVERYTHING)
        draw_logo()
        return
    else:
      self.dl_ready = False

    # Draw the home cube with a gray floor

    if glIsList(LIST_FURNITURE):
      glCallList(LIST_FURNITURE)
    else:
      glNewList(LIST_FURNITURE, GL_COMPILE_AND_EXECUTE)
      glColor3f(1, 1, 1)
      draw_cube()
      glColor3f(.6, .6, .6)
      glBegin(GL_QUADS)
      for x,z in [ (-1,-1), (-1,1), (1,1), (1,-1) ]:
        glVertex3f(x, -1, z)
      glEnd()
      glBegin(GL_LINES)
      for x in range(-100, 100):
        for z in range(-100, 100):
          glVertex3f(x-.1, -1, z)
          glVertex3f(x+.1, -1, z)
          glVertex3f(x, -1, z+.1)
          glVertex3f(x, -1, z-.1)
      glEnd()
      glEndList()

    # Draw the snail trail

    if True: # demo.connected:
      anchor = demo.anchor()
      if demo.connected:
        glColor3f(0, 1, 0)
      else:
        glColor3f(1, 0, 0)
      glBegin(GL_LINE_STRIP)
      for p in demo.snail_trail:
        relpose = anchor * (~demo.snail_trail[0] * p)
        x,y,z = relpose.xform(0,0,0)
        glVertex3f(-x, -y, z)
      glEnd()

    # Draw the skeleton edges as thick lines

    def label2color(l):
      # most recent N get N distinctive colors
      # older labels get the same color
      fresh_colors = [
        (153,255,0),
        (255,230,0),
        (255,102,0) ]
      old_color = (153,102,0)

      if (demo.label - l) < len(fresh_colors):
        return fresh_colors[l % len(fresh_colors)]
      else:
        return old_color

    glLineWidth(3)

    edges_plain = {}  # edges that are the same color
    edges_cross = []  # cross track edges
    edges_weak = []

    for i0,i1 in demo.skel.edges:
      l0 = demo.skel.node_labels[i0]
      l1 = demo.skel.node_labels[i1]
      if l0 == l1:
        edges_plain.setdefault(l0, [])
        edges_plain[l0].append((i0,i1))
      else:
        if (i0,i1) in demo.skel.weak_edges:
          edges_weak.append((i0,i1))
        else:
          edges_cross.append((i0,i1))

    xformed = dict([(i, demo.skel.newpose(i).xform(0,0,0)) for i in demo.skel.nodes])

    def draw_edge(i0,i1):
      (x,y,z) = xformed[i0]
      glVertex3f(-x,-y,z)
      (x,y,z) = xformed[i1]
      glVertex3f(-x,-y,z)

    glLineWidth(3)

    for l,vv in edges_plain.items():
      glColor3ub(*label2color(l))
      glBegin(GL_LINES)
      for (i0,i1) in vv:
        draw_edge(i0, i1)
      glEnd()

    glColor3f(.3, .3, 1)
    glBegin(GL_LINES)
    for i0,i1 in edges_cross:
      draw_edge(i0, i1)
    glEnd()

    glLineStipple(3, 0x5555)
    glEnable(GL_LINE_STIPPLE)
    glBegin(GL_LINES)
    for i0,i1 in edges_weak:
      draw_edge(i0, i1)
    glEnd()
    glDisable(GL_LINE_STIPPLE)
      
    glLineWidth(1)
      
    # Draw the skeleton nodes as frustums

    for id in demo.skel.nodes:
      l0 = demo.skel.node_labels[id]
      glColor3ub(*label2color(l0))

      glPushMatrix()
      poseMultMatrix(demo.skel.newpose(id))
      draw_view()
      glPopMatrix()

    if 0:
      glTranslatef(0, 0.5, 0)
    else:
      poseMultMatrix(demo.pose())

    glColor3f(1, 1, 1)
    draw_camera()

    if demo.running == 0 and not self.dl_ready:
      glEndList()
      self.dl_ready = True

    draw_logo()

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
    self.mode = 1
    self.fullscreen = False
    self.th = 0
    self.phi = -math.pi / 2

  def matrix(self):
    if self.mode == 0:
      R = transformations.euler_matrix(self.phi, self.th, 0)
    else:
      R = transformations.euler_matrix(self.phi, 0, self.th)
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

    damp_weak = math.pow(0.99, delta)
    damp_strong = math.pow(0.01, delta)
    self.v_fwd += delta * self.a_fwd
    if self.a_fwd:
      self.v_fwd *= damp_weak
    else:
      self.v_fwd *= damp_strong

    if self.mode == 0:
      self.v_rgt += delta * self.a_rgt
      if self.a_rgt:
        self.v_rgt *= damp_weak
      else:
        self.v_rgt *= damp_strong

      self.v_clm += delta * self.a_clm
      if self.a_clm:
        self.v_clm *= damp_weak
      else:
        self.v_clm *= damp_strong

      self.translate(self.v_rgt * delta, self.v_clm * delta, -self.v_fwd * delta)
    else:
      x,y,z = demo.pose().xform(0,0,0)
      self.x = -x
      self.y -= self.v_fwd * delta
      self.z = z

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

    if k == 'o':
      demo.optimize()

    if k == 'i':
      demo.ioptimize()

    if k == '\r':
      self.fullscreen = not self.fullscreen
      if self.fullscreen:
        glutFullScreen()
      else:
        glutPositionWindow(0, 0)
        glutReshapeWindow(640, 480 + BOTTOM_2D_HEIGHT)

    if k == '/':
      demo.report()

    if k == 'S':
      print "SAVING\n"
      time.sleep(1)
      demo.skel.pg.save("demo_snapshot.toro")
      time.sleep(1)

    if k == 'P':
      print "\nPAUSING\n"
      if demo.running:
        demo.running = 0
      else:
        demo.running = -1

    if k == 'p':
      print "\nSINGLE STEP\n"
      demo.running = 1

    if k == 'm':
      self.mode = (self.mode + 1) % 2
      if self.mode == 1:
        self.y = 10
        self.th = 0
        self.phi = -math.pi / 2
    if self.mode == 0:
      glutWarpPointer(320,240)
      glutSetCursor(GLUT_CURSOR_NONE)
    else:
      glutSetCursor(GLUT_CURSOR_INHERIT)

    factor = demo.map_size()

    th_inc = 0.05
    if k == 'w':
      self.a_fwd = 1 * factor
    elif k == 's':
      self.a_fwd = -1 * factor
    elif k == 'a':
      self.a_rgt = -1 * factor
    elif k == 'd':
      self.a_rgt = 1 * factor
    elif k == ' ':
      self.a_clm = 1 * factor
    elif k == 'c':
      self.a_clm = -1 * factor

  def zoom(self, d):
    if self.mode == 1:
      self.y += d
    else:
      self.v_fwd = -7 * d

  def passive(self, x, y):
    if self.mode == 0:
      if x != 320 or y != 240:
        self.turn_r((x - 320) * 0.001)
        self.turn_u((y - 240) * 0.001)
        glutWarpPointer(320,240)

  def start_drag(self, x, y):
    self.drag_th = self.th
    self.drag_phi = self.phi
    self.drag_x = x
    self.drag_y = y

  def stop_drag(self, x, y):
    pass

  def drag(self, x, y):
    self.th = self.drag_th + -0.001 * (x - self.drag_x)
    self.phi = self.drag_phi + 0.001 * (y - self.drag_y)

ml = MouseLook(0,10,4)

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

def mouse(button, state, x, y):
  if button == 3 and state == 1:
    ml.zoom(-1)
  if button == 4 and state == 1:
    ml.zoom(1)
  if button == 0:
    if state == 0:
      ml.start_drag(x, y)
    else:
      ml.stop_drag(x, y)

def mouse_motion(x, y):
  ml.drag(x, y)

def every():
  ml.frob()
  updateMV()
  demo.redraw()

import Image
import ImageFont, ImageDraw

if len(sys.argv) == 1:
  demo = GLDemo(RealSource())
else:
  demo = GLDemo(FakeSource(sys.argv[1]))

def main():

  global window
  glutInit(sys.argv)

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)

  glutInitWindowSize(640, 480 + BOTTOM_2D_HEIGHT)

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
  glutMouseFunc(mouse)
  glutMotionFunc(mouse_motion)
  glutIgnoreKeyRepeat(True)
  glutPassiveMotionFunc(ml.passive)

  InitGL(640, 480 + BOTTOM_2D_HEIGHT)
  updateMV()

  demo.textures = glGenTextures(100)

  for h in demo.textures:
    glBindTexture(GL_TEXTURE_2D, h)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

  for i,txt in [ (0, "M"), (1, "A") ]:
    glBindTexture(GL_TEXTURE_2D, demo.textures[i])
    im = Image.new("L", (1024, 1024))
    draw = ImageDraw.Draw(im)
    font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 1024-64)
    (w,h) = font.getsize(txt)
    draw.pieslice((32,32,1024-32,1024-32), 0, 360, fill = 255)
    draw.text(((1024-w)/2, (1024-h)/2+100), txt, font=font, fill=0)
    im32 = im.resize((512, 512), Image.ANTIALIAS).resize((256, 256), Image.ANTIALIAS).resize((128, 128), Image.ANTIALIAS).resize((64, 64), Image.ANTIALIAS).resize((32, 33), Image.ANTIALIAS)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE8, 32, 32, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, im32.tostring())

  glutMainLoop()

print "-" * 80
print
print "ENTER for fullscreen"
print "'m' toggles manual camera"
print "'o' run optimization"
print
print "Hit ESC key to quit."
print
print "-" * 80

main()
