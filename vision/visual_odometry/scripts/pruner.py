#!/usr/bin/python

import rostools
rostools.update_path('visual_odometry')
import rostest

import sys
sys.path.append('lib')

import visual_odometry as VO
import rosrecord
import rospy

import Image as Image

import camera
from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, FeatureDetectorFast, FeatureDetectorStar, FeatureDetectorHarris, DescriptorSchemeCalonder, DescriptorSchemeSAD

import numpy
import pylab

def load_from_bag(filename, selected_frames):
  cam = None
  framecounter = 0
  afs = {}
  for topic, msg in rosrecord.logplayer(filename):
    if rospy.is_shutdown():
      break

    if topic == "/videre/cal_params" and not cam:
      cam = camera.VidereCamera(msg.data)

    if cam and topic == "/videre/images":
      print "frame", framecounter
      if framecounter in selected_frames:
        def imgAdapted(msg_img):
          return Image.fromstring("L", (msg_img.width, msg_img.height), msg_img.data)
        imgR = imgAdapted(msg.images[0])
        imgL = imgAdapted(msg.images[1])
        afs[framecounter] = SparseStereoFrame(imgL, imgR)
        if framecounter == max(selected_frames):
          break
      framecounter += 1
  return (cam, afs)

def load_from_dir(dir, selected_frames):
  afs = dict([(f, SparseStereoFrame(Image.open("%s/left-%04d.ppm" % (dir,f)), Image.open("%s/right-%04d.ppm" % (dir,f)))) for f in selected_frames])
  return afs

########################################################################

import pickle

f = open("pruned.pickle", "r")
cam = pickle.load(f)
db = pickle.load(f)
f.close()

class LibraryFrame:
  def __init__(self, pose, kp, desc):
    self.pose = pose
    self.kp = kp
    self.descriptors = desc

afs = {}
for (id, pose, kp, desc) in db:
  lf = LibraryFrame(pose, kp, desc)
  afs[id] = lf

vo = VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeSAD())

print "%d frames in library" % len(afs)
most_overlap = 99999
while most_overlap > 180:
  ov = []
  for i in afs.keys():
    for j in [k for k in afs.keys() if k > i]:
      (overlap, _) = vo.proximity(afs[i], afs[j])
      ov.append((overlap, i))
  print max(ov)
  most_overlap = max(ov)[0]
  del afs[max(ov)[1]]
print "%d frames in library" % len(afs)

f = open("pruned.pickle", "w")
pickle.dump(cam, f)
db = [(i, afs[i].pose, afs[i].kp, afs[i].descriptors) for i in afs.keys()]
pickle.dump(db, f)
f.close()

vos = [ vo ]

def metric(af0, af1):
  """ Given a pair of frames, return the distance metric using inliers from the solved pose """
  pairs = vo.temporal_match(af0, af1)
  if len(pairs) > 10:
    (inl, rot, shift) = vo.solve(af0.kp, af1.kp, pairs)
    if inl == 0:
      return 0
    else:
      return min(200, inl)
  else:
    return 0

fig = pylab.figure()
img1_ax = fig.add_subplot(222)
img2_ax = fig.add_subplot(224)

def onclick(event):
    index1 = int(event.ydata)
    index2 = int(event.xdata)
    display_images(index1, index2)

for (mode_i, vo) in enumerate(vos):
  keys = sorted(afs.keys())

  results = []
  for i in keys:
    line = [ metric(afs[i], afs[j]) for j in keys if j <= i ] + ([0] * len([j for j in keys if j > i]))
    results.append(line)

  pylab.figure(2 + mode_i)
  pylab.pcolor(numpy.array(results))
  pylab.colorbar()
  pylab.connect('button_press_event', onclick)
  pylab.title("%s %s" % (vo.feature_detector.name(), vo.descriptor_scheme.name()))

  def display_images(index1, index2):
      pylab.figure(1)
      for ax,index in ([img1_ax, index1], [img2_ax, index2]):
        ax.imshow(pylab.asarray(afs[keys[index]].lf), cmap=pylab.cm.gray, interpolation='nearest')
        ax.set_title('Frame %d' % keys[index])
      pylab.draw()

pylab.show()
