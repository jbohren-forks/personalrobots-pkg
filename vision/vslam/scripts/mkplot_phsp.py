#!/usr/bin/python

import rostools
rostools.update_path('vslam')
import rostest
import rospy

import vop

import sys

import visual_odometry as VO
import Image as Image
import ImageChops as ImageChops
import ImageDraw as ImageDraw
import ImageFilter as ImageFilter

import random
import unittest
import math
import copy

from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
from skeleton import Skeleton

from math import *

import camera

import numpy
import numpy.linalg
import pylab

import rosrecord
import transformations

from vis import Vis

import pickle

def planar(x, y, z):
  from scipy import optimize

  def silly(sol, args):
      a,b,c = sol
      x,y,z = args
      return sum((y - (a*x + b*z + c)) ** 2)

  sol = [1.0, 1.0, 1.0]
  sol = optimize.fmin(silly, sol, args=((x,y,z),))

  a,b,c = sol
  return sqrt(sum((y - (a*x + b*z + c)) ** 2) / len(x))

class dcamImage:
  def __init__(self, m):
    if hasattr(m, "byte_data"):
      ma = m.byte_data
      self.data = ma.data
    else:
      ma = m.uint8_data # MultiArray
      self.data = "".join([chr(x) for x in ma.data])
    d = ma.layout.dim
    assert d[0].label == "height"
    assert d[1].label == "width"
    self.size = (d[1].size, d[0].size)
    self.mode = "L"

  def tostring(self):
    return self.data

cam = None
filename = "/u/prdata/videre-bags/loop2-color.bag"
filename = "/u/prdata/videre-bags/greenroom-2008-11-3-color.bag"
filename = "2008-11-04-09-55-12-topic.bag"
filename = "2008-11-04-14-44-56-topic.bag"
filename = "2008-11-05-14-35-11-topic.bag"
filename = "/u/prdata/videre-bags/vo1.bag"
filename = sys.argv[1]
framecounter = 0
first_pair = None
inliers = []

if 0:
  start,end = 20000,21300
  skipto = 12634364243
  start,end = 0,1300
else:
  skipto = None
  start,end = 0, 10000

print "starting loop"
f = open(filename)
for topic, msg, timeStamp in rosrecord.logplayer(filename):
  print 'topic:',topic
  if skipto and (f.tell() < skipto):
    f.seek(skipto)
  #print f.tell(), msg
  if rospy.is_shutdown():
    break

  if topic.endswith("stereo/raw_stereo") or topic.endswith("dcam/raw_stereo"):
    if not cam:
      cam = camera.StereoCamera(msg.right_info)
      vos = [
        VisualOdometer(cam, scavenge = True, 
                       #feature_detector = FeatureDetectorFast(), 
                       feature_detector = FeatureDetectorHarris(), 
                       inlier_error_threshold = 3.0, 
                       sba = (10,10,10),
                       # sba = None,
                       inlier_thresh = 100,
                       position_keypoint_thresh = 0.2, angle_keypoint_thresh = 0.15)
      ]
      vo_x = [ [] for i in vos]
      vo_y = [ [] for i in vos]
      vo_u = [ [] for i in vos]
      vo_v = [ [] for i in vos]
      trajectory = [ [] for i in vos]
      stampedTrajectory = [ [] for i in vos]
      skel = Skeleton(cam)
      oe_x = []
      oe_y = []
      oe_home = None
      stampedGroundTruth = []
    if framecounter == end:
      break
    if start <= framecounter and (framecounter % 1) == 0:
      for i,vo in enumerate(vos):
        af = SparseStereoFrame(dcamImage(msg.left_image), dcamImage(msg.right_image))
        
        # jdc debugging
        Image.fromstring("L", af.size, af.rawdata).save("/tmp/mkplot-left.png")   
             
        vo.handle_frame(af)
        if i == 0:
          skel.add(vo.keyframe)
        x,y,z = vo.pose.xform(0,0,0)
        trajectory[i].append((x,y,z))
        # convert the time stampe to milliseconds in float
        secs = timeStamp.secs + timeStamp.nsecs*1.e-9
        stampedTrajectory[i].append((secs, x, y, z))
        vo_x[i].append(x)
        vo_y[i].append(z)
        x1,y1,z1 = vo.pose.xform(0,0,1)
        vo_u[i].append(x1 - x)
        vo_v[i].append(z1 - z)
      print framecounter, vo.inl, "inliers"
      inliers.append(vo.inl)
    framecounter += 1

  def ground_truth(p, q):
    return Pose(transformations.rotation_matrix_from_quaternion([q.x, q.y, q.z, q.w])[:3,:3], [p.x, p.y, p.z])

  gtp = None
  if topic.endswith("odom_estimation"):
    gtp = ground_truth(msg.pose.position, msg.pose.orientation)
    ground_truth_label = "PhaseSpace"
  if topic.endswith("phase_space_snapshot"):
    gtp = ground_truth(msg.bodies[0].pose.translation, msg.bodies[0].pose.rotation)
    ground_truth_label = "wheel + IMU odometry"
  if gtp and cam:
    oe_pose = gtp
    if not oe_home:
      oe_home = oe_pose
    local = ~oe_home * oe_pose
    (x,y,z) = local.xform(0,0,0) # ground truth x y z
    # note that the phase space frame is x forward, y to left and z upward.
    oe_x.append(-y)
    oe_y.append(x)
    # convert the time stamp to seconds in float
    secs = timeStamp.secs + timeStamp.nsecs*1.e-9
    stampedGroundTruth.append((secs, -y, -z, x))

print "There are", len(vo.tracks), "tracks"
print "There are", len([t for t in vo.tracks if t.alive]), "live tracks"
print "There are", len(set([t.p[-1] for t in vo.tracks if t.alive])), "unique endpoints on live tracks"

quality_pose = Pose()
if 0:
  # Attempt to compute best possible end-to-end pose
  vo = VisualOdometer(cam, feature_detector = FeatureDetector4x4(FeatureDetectorHarris), scavenge = True)
  f0 = SparseStereoFrame(*first_pair)
  f1 = SparseStereoFrame(imgL, imgR)
  vo.handle_frame(f0)
  vo.handle_frame(f1)
  quality_pose = vo.pose

if 0:
  for t in vos[2].all_tracks:
    pylab.plot([x for (x,y,d) in t.p], [y for (x,y,d) in t.p])
  pylab.xlim((0, 640))
  pylab.ylim((0, 480))
  pylab.savefig("foo.png", dpi=200)
  pylab.show()
  sys.exit(0)

pylab.figure(figsize=(20,20))
colors = [ 'red', 'black', 'magenta', 'cyan', 'orange', 'brown', 'purple', 'olive', 'gray' ]
  
for i in range(len(vos)):
  vos[i].planarity = planar(numpy.array([x for (x,y,z) in trajectory[i]]), numpy.array([y for (x,y,z) in trajectory[i]]), numpy.array([z for (x,y,z) in trajectory[i]]))
  xs = numpy.array(vo_x[i])
  ys = numpy.array(vo_y[i])
  if 0:
    xs -= 4.5 * 1e-3
    f = -0.06
  else:
    f = 0.0
  xp = xs * cos(f) - ys * sin(f)
  yp = ys * cos(f) + xs * sin(f)
  pylab.plot(xp, yp, c = colors[i], label = vos[i].name())
  #pylab.quiver(xp, yp, vo_u[i], vo_v[i], color = colors[i]) #, label = '_nolegend_')

  #xk = [ x for j,x in enumerate(vo_x[i]) if j in vos[i].log_keyframes ]
  #yk = [ y for j,y in enumerate(vo_y[i]) if j in vos[i].log_keyframes ]
  #pylab.scatter(xk, yk, c = colors[i], label = '_nolegend_')

pylab.plot(oe_x, oe_y, c = 'green', label = ground_truth_label)

#skel.optimize()
skel.plot('blue')

xlim = pylab.xlim()
ylim = pylab.ylim()
xrange = xlim[1] - xlim[0]
yrange = ylim[1] - ylim[0]
r = max(xrange, yrange) * 0.75
mid = sum(xlim) / 2
pylab.xlim(mid - r, mid + r)
mid = sum(ylim) / 2
pylab.ylim(mid - r, mid + r)
pylab.legend()
pylab.savefig("foo.png", dpi=200)
pylab.show()

#pickling
output_trajectory = open('trajs.pkl','wb')
output_gt         = open('gt.pkl', 'wb')
pickle.dump(stampedTrajectory,  output_trajectory)
pickle.dump(stampedGroundTruth, output_gt)
output_trajectory.close()
output_gt.close()

# extract the position of the key frames and pickle them
stampedKeyFrameTrajectory = [ [] for i in vos]
for i,vo in enumerate(vos):
  k=0;
  for keyframeId in vo.log_keyframes:
    print 'keyframe id', keyframeId,i,k
    stampedKeyFrameTrajectory[i].append(stampedTrajectory[i][keyframeId])
    k+=1
    
output_keyframe_trajectory = open('keyframe_trajs.pkl','wb')
pickle.dump(stampedKeyFrameTrajectory, output_keyframe_trajectory)
output_keyframe_trajectory.close()

for vo in vos:
  print vo.name()
  print "distance from start:", vo.pose.distance()
  print "planarity", vo.planarity
  print "pose", vo.pose.comparison(quality_pose)
  vo.summarize_timers()
  print vo.log_keyframes
  print
