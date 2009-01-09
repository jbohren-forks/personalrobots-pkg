import rostools
rostools.update_path('vslam')

import Image
from votools import TreeOptimizer3
import place_recognition
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler
from stereo import SparseStereoFrame
from timer import Timer

import pylab, numpy

class minimum_frame:
  def __init__(self, kp, descriptors, matcher):
    self.kp = kp
    self.descriptors = descriptors
    self.matcher = matcher
    assert len(kp) == len(descriptors)

class Skeleton:
  def __init__(self, cam):
    self.nodes = set()
    self.edges = set()
    self.pg = TreeOptimizer3()
    self.pg.initializeOnlineOptimization()
    if 1:
      self.vt = place_recognition.vocabularytree()
      ims = [Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (200 * i)) for i in range(10)]
      #ims = [Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (20 * i)) for i in range(100)]
      self.vt.build(ims, 5, 4, False)
    else:
      self.vt = None
    self.place_ids = []
    #self.vo = VisualOdometer(cam)
    self.vo = VisualOdometer(cam, feature_detector = FeatureDetectorStar(), descriptor_scheme = DescriptorSchemeCalonder())
    self.node_kp = {}
    self.node_descriptors = {}
    self.node_matcher = {}
    self.termcrit = lambda count, err: (count > 5) or (err < 1.0)

    self.timer = {}
    for t in ['toro add', 'toro opt', 'pr add', 'pr search', 'gcc', 'descriptors']:
      self.timer[t] = Timer()

  def add(self, this):
    if len(self.nodes) == 0:
      self.nodes.add(this)
    elif not(this in self.nodes):
      byid = [ (f.id, f) for f in self.nodes ]
      prev = max(byid)[1]
      relpose = ~prev.pose * this.pose

      self.nodes.add(this)
      self.edges.add( (prev, this) )
      self.timer['toro add'].start()
      self.pg.addIncrementalEdge(prev.id, this.id, relpose.xform(0,0,0), relpose.euler())
      self.timer['toro add'].stop()
      #print "added node at", this.pose.xform(0,0,0), "in graph as", self.newpose(this).xform(0,0,0)

      self.memoize_node_kp_d(this)
      this_d = self.node_descriptors[this.id]
      if len(self.nodes) > 1:
        far = [ f for f in self.place_find(this.lf, this_d, 10) if (not f.id in [this.id, prev.id])]
        self.add_links(this, far)

      self.timer['pr add'].start()
      if self.vt:
        self.vt.add(this.lf, this_d)
      self.timer['pr add'].stop()
      self.place_ids.append(this)

  def addConstraint(self, prev, this, relpose):
    self.edges.add((prev, this))
    self.timer['toro add'].start()
    self.pg.addIncrementalEdge(prev.id, this.id, relpose.xform(0,0,0), relpose.euler())
    self.timer['toro add'].stop()

  def optimize(self):
    self.pg.initializeOnlineIterations()
    print "pg.error", self.pg.error()
    for i in range(1000):
      #print "iter", i, "pg.error", self.pg.error()
      self.pg.iterate()
    print "pg.error", self.pg.error()
    #self.pg.recomputeAllTransformations()
    #self.pg.save("render5.graph")

  def newpose(self, f):
    xyz,euler = self.pg.vertex(f.id)
    return from_xyz_euler(xyz, euler)

  def place_find(self, lf, descriptors, count = 10):
    if self.vt:
      self.timer['pr search'].start()
      scores = self.vt.topN(lf, descriptors, count)
      self.timer['pr search'].stop()
      assert len(scores) == len(self.place_ids)
      return [id for (_,id) in sorted(zip(scores, self.place_ids), reverse=True)][:count]
    else:
      return self.place_ids

  def add_links(self, this, far):
    self.timer['gcc'].start()
    coll = [ self.PE(this, f) + (f,) for f in far ]
    self.timer['gcc'].stop()
    id0 = this
    for inl,obs,id1 in coll:
      if 100 <= inl:
        self.addConstraint(id0, id1, obs)
        print "ADDING CONSTRAINT"
    self.timer['toro opt'].start()
    self.pg.initializeOnlineIterations()
    count = 0
    while not self.termcrit(count, self.pg.error()):
      self.pg.iterate()
      count += 1
    this.pose = self.newpose(this)
    self.timer['toro opt'].stop()

  def my_frame(self, id):
    return minimum_frame(self.node_kp[id], self.node_descriptors[id], self.node_matcher[id])

  def memoize_node_kp_d(self, af):
    self.timer['descriptors'].start()
    if not (af.id in self.node_kp):
      nf = SparseStereoFrame(af.lf, af.rf)
      self.vo.setup_frame(nf)
      self.node_kp[af.id] = nf.kp
      self.node_descriptors[af.id] = nf.descriptors
      self.node_matcher[af.id] = nf.matcher
    self.timer['descriptors'].stop()

  def PE(self, af0, af1):
    return self.vo.proximity(self.my_frame(af0.id), self.my_frame(af1.id))

  def plot(self, color):
    pts = dict([ (f,self.newpose(f).xform(0,0,0)) for f in self.nodes ])
    nodepts = pts.values()
    pylab.scatter([x for (x,y,z) in nodepts], [z for (x,y,z) in nodepts], c = color, label = 'after SGD')
    for (f0,f1) in self.edges:
      p0 = pts[f0]
      p1 = pts[f1]
      pylab.plot((p0[0], p1[0]), (p0[2], p1[2]), c = color)

  def average_time_per_frame(self):
    niter = len(self.nodes)
    return 1e3 * sum([t.sum for t in self.timer.values()]) / niter

  def summarize_timers(self):
    niter = len(self.nodes)
    if niter != 0:
      for n,t in self.timer.items():
        print "%-20s %fms" % (n, 1e3 * t.sum / niter)
      print "%-20s %fms" % ("TOTAL", self.average_time_per_frame())
    #self.vo.summarize_timers()
