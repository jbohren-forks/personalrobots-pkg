import rostools
rostools.update_path('vslam')

import Image
from votools import TreeOptimizer3
import place_recognition
import camera
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler
from stereo import SparseStereoFrame
from timer import Timer

import pylab, numpy
import cPickle as pickle

import math
import random
import time
random.seed(0)

pg = TreeOptimizer3()
pg.initializeOnlineOptimization()

cam = camera.Camera((432.0, 432.0, 0.088981018518518529, 313.78210000000001, 313.78210000000001, 220.40700000000001))
#vo = VisualOdometer(cam)
vo = VisualOdometer(cam, scavenge = True, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeCalonder())

def pg_constraint(pg, a, b, pose, inf):
  pg.addIncrementalEdge(a, b, pose.xform(0,0,0), pose.euler(), inf)

def newpose(id):
  xyz,euler = pg.vertex(id)
  return from_xyz_euler(xyz, euler)

def mk_covar(xyz, rp, yaw):
  return (1.0 / math.sqrt(xyz),1.0 / math.sqrt(xyz), 1.0 / math.sqrt(xyz), 1.0 / math.sqrt(rp), 1.0 / math.sqrt(rp), 1.0 / math.sqrt(yaw))

# 0 is a weak link, 1 is strong

weak = mk_covar(0.01, 0.0002, 0.002)
strong = mk_covar(0.0001, 0.000002, 0.00002)

def lerp(t, a, b):
  return a + (b - a) * t

def link_strength(t):
  assert 0 <= t
  assert t <= 1
  return mk_covar(lerp(t, 0.01, 0.0001), lerp(t, 0.0002, 0.000002), lerp(t, 0.002, 0.00002))

if 0:
  pg_constraint(pg, 0, 1, Pose(numpy.identity(3), (1,0,0)), link_strength(0))
  pg_constraint(pg, 1, 2, Pose(numpy.identity(3), (1,0,0)), link_strength(0))
  pg_constraint(pg, 0, 2, Pose(numpy.identity(3), (1,0,0)), link_strength(1))

  pg.initializeOnlineIterations()
  print "pg.error", pg.error()
  for i in range(1000):
    pg.iterate()
  print "pg.error", pg.error()

  print 0, newpose(0).xform(0,0,0)
  print 1, newpose(1).xform(0,0,0)
  print 2, newpose(2).xform(0,0,0)
  sys.exit(1)

if 0:
  vt = None
else:
  if 0:
    vt = place_recognition.vocabularytree()
    ims = [Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (200 * i)) for i in range(10)]
    vt.build(ims, 5, 4, False)
  else:
    vt = place_recognition.load("/u/mihelich/images/holidays/holidays.tree")

f = []
for i in range(180): # range(1,146):
  print i
  dir = "/u/jamesb/ros/ros-pkg/vision/vslam/pool_loop"
  L = Image.open("%s/%06dL.png" % (dir, i))
  R = Image.open("%s/%06dR.png" % (dir, i))
  nf = SparseStereoFrame(L, R)
  vo.setup_frame(nf)
  nf.id = len(f)
  if vt:
    vt.add(nf.lf, nf.descriptors)
  f.append(nf)

# Write confusion matrix to file - for Patrick
if 0:
  confusion = [ [ vo.proximity(a, b, True)[0] for a in f ] for b in f ]
  pylab.pcolor(numpy.array(confusion))
  pylab.show()
  txt = open("confusion-inliers.txt", "w")
  for l in confusion:
    print >>txt, l
  txt.close()
  sys.exit(0)

if 0:
  for i in range(146):
    print i, i + 1, vo.proximity(f[i], f[i+1], True)
  sys.exit(0)

all_ids = set(range(len(f)))

focus = 0

nodes = set()
edges = set()

nodes.add(focus)

def snap(movie, all_ids, checked, nodes, edges, error):
  return
  filename = 'z%06d.pickle' % movie
  f = open(filename, "w")
  pickle.dump(all_ids, f)
  pickle.dump(checked, f)
  pts = dict([ (id,newpose(id).xform(0,0,0)) for id in nodes ])
  pickle.dump(pts, f)
  pickle.dump(edges, f)
  pickle.dump(error, f)
  f.close()

checked = set()
movie = 0
highest_scores = []
started = time.time()
while True:
  print "*** focus ***", focus
  count = len(f)
  to_search = all_ids - set([focus]) - checked
  if vt:
    scores = vt.topN(f[focus].lf, f[focus].descriptors, count)
    if 0:
      for (sc, id) in  sorted(zip(scores, range(count)), reverse=False):
        print id, sc
      print
    best = [id for (score,id) in sorted(zip(scores, range(count))) if (id in to_search)][:20]
  else:
    best = [id for id in all_ids if id in to_search]
  prox = sorted([ vo.proximity(f[focus], f[near], True) + (near,) for near in best ], reverse=True)
  inl_thresh = 30
  gpass = [ (inl,pose,near) for (inl,pose,near) in prox if (inl > inl_thresh) ][:8]
  #if gpass != []: highest_scores.append(max([scores[n] for (_,_,n) in gpass]))
  for inl,pose,near in gpass:
    t = min((inl - inl_thresh) / 150., 1.0)
    edges.add((focus, near, t))
    nodes.add(near)
    pg.addIncrementalEdge(focus, near, pose.xform(0,0,0), pose.euler(), link_strength(t))
    print "edge from", focus, near

  checked.add(focus)
  if nodes == checked:
    break
  #print "About to sample", "nodes", sorted(list(nodes)), "checked", sorted(list(checked))
  focus = random.sample(nodes - checked, 1)[0]
  snap(movie, all_ids, checked, nodes, edges, pg.error())
  movie += 1

print "Leftovers:", all_ids - checked
#print max(highest_scores)
#sys.exit(1)

pg.initializeOnlineIterations()
print "pg.error", pg.error()
for i in range(120):
  pg.iterate()
  #print i, "pg.error", pg.error()
  snap(movie, all_ids, checked, nodes, edges, pg.error())
  movie += 1
print "pg.error", pg.error()
print "Took:", time.time() - started

if 1:
  fig = pylab.figure(figsize=(7,7))
  pts = dict([ (id,newpose(id).xform(0,0,0)) for id in nodes ])
  nodepts = pts.values()
  pylab.scatter([x for (x,y,z) in nodepts], [z for (x,y,z) in nodepts], c = 'blue')
  for (id,(x,y,z)) in pts.items():
    pylab.annotate('%d' % id, (float(x), float(z)))
  for (f0,f1,t) in edges:
    p0 = pts[f0]
    p1 = pts[f1]
    r = int(lerp(t, 250,  50))
    g = int(lerp(t, 250,  50))
    b = int(lerp(t, 250, 250))
    color = '#%02x%02x%02x' % (r,g,b)
    pylab.plot((p0[0], p1[0]), (p0[2], p1[2]), c = color)
  pylab.xlim(-2, 12)
  pylab.ylim(-2, 12)
  pylab.savefig("zero%06d.png" % i, dpi=200)
  #pylab.show()
  pylab.close(fig)

if 1:
  xlim = pylab.xlim()
  ylim = pylab.ylim()
  xrange = xlim[1] - xlim[0]
  yrange = ylim[1] - ylim[0]
  r = max(xrange, yrange) * 0.5
  mid = sum(xlim) / 2
  pylab.xlim(mid - r, mid + r)
  mid = sum(ylim) / 2
  pylab.ylim(mid - r, mid + r)
