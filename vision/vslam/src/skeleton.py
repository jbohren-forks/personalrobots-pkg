import rostools
rostools.update_path('vslam')

import Image
from votools import TreeOptimizer3
import place_recognition
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler
from stereo import SparseStereoFrame
import pylab, numpy

class minimum_frame:
  def __init__(self, kp, descriptors, matcher):
    self.kp = kp
    self.descriptors = descriptors
    self.matcher = matcher
    assert len(kp) == len(descriptors)
    print matcher

class Skeleton:
  def __init__(self, cam):
    self.nodes = set()
    self.edges = set()
    self.pg = TreeOptimizer3()
    self.pg.initializeOnlineOptimization()
    if 1:
      self.vt = place_recognition.vocabularytree()
      ims = [Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (200 * i)) for i in range(10)]
      self.vt.build(ims, 5, 4, False)
    else:
      self.vt = None
    self.place_ids = []
    self.vo = VisualOdometer(cam, feature_detector = FeatureDetectorStar(), descriptor_scheme = DescriptorSchemeCalonder())
    self.node_kp = {}
    self.node_descriptors = {}
    self.node_matcher = {}
    self.termcrit = lambda count, err: (count > 100) or (err < 0.1)

  def add(self, this):
    if len(self.nodes) == 0:
      self.nodes.add(this)
    elif not(this in self.nodes):
      byid = [ (f.id, f) for f in self.nodes ]
      prev = max(byid)[1]
      relpose = ~prev.pose * this.pose

      self.nodes.add(this)
      self.edges.add( (prev, this) )
      self.pg.addIncrementalEdge(prev.id, this.id, relpose.xform(0,0,0), relpose.euler())
      #print "added node at", this.pose.xform(0,0,0), "in graph as", self.newpose(this).xform(0,0,0)

      # XXX - waiting for fix from Patrick
      if len(self.nodes) > 20:
        #far = [ f for f in self.place_find(this.lf, 10) if abs(f.id - this.id) > 120]
        far = [ f for f in self.place_find(this.lf, 10) if (f.id != this.id)]
        self.add_links(this, far)

      if self.vt:
        self.vt.add(this.lf)
      self.place_ids.append(this)

  def addConstraint(self, prev, this, relpose):
    self.edges.add((prev, this))
    self.pg.addIncrementalEdge(prev.id, this.id, relpose.xform(0,0,0), relpose.euler())

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

  def place_find(self, f, count = 10):
    if self.vt:
      scores = self.vt.topN(f, count)
      assert len(scores) == len(self.place_ids)
      return [id for (_,id) in sorted(zip(scores, self.place_ids), reverse=True)][:count]
    else:
      return self.place_ids

  def add_links(self, this, far):
    coll = [ self.PE(this, f) + (f,) for f in far ]
    id0 = this
    for inl,obs,id1 in coll:
      if 100 <= inl:
        self.addConstraint(id0, id1, obs)
        print "ADDING CONSTRAINT"
    self.pg.initializeOnlineIterations()
    count = 0
    while not self.termcrit(count, self.pg.error()):
      self.pg.iterate()
      count += 1
    this.pose = self.newpose(this)

  def my_frame(self, id):
    return minimum_frame(self.node_kp[id], self.node_descriptors[id], self.node_matcher[id])

  def PE(self, af0, af1):
    for af in (af0,af1):
      if not (af.id in self.node_kp):
        nf = SparseStereoFrame(af.lf, af.rf)
        self.vo.setup_frame(nf)
        self.node_kp[af.id] = nf.kp
        self.node_descriptors[af.id] = nf.descriptors
        self.node_matcher[af.id] = nf.matcher
    prox = self.vo.proximity(self.my_frame(af0.id), self.my_frame(af1.id))
    if af0.id == 97 and af1.id == 92:
      af0.lf.save("f0-left.png")
      af0.rf.save("f0-right.png")
      af1.lf.save("f1-left.png")
      af1.rf.save("f1-right.png")
      print "attempting match of", af0.id, af1.id
      f0 = self.my_frame(af0.id)
      f1 = self.my_frame(af1.id)
      pairs = self.vo.temporal_match(f0, f1)
      for (a,b) in pairs:
        pylab.plot([ f0.kp[a][0], f1.kp[b][0] ], [ f0.kp[a][1], f1.kp[b][1] ])
      pylab.imshow(numpy.fromstring(af0.lf.tostring(), numpy.uint8).reshape(480,640), cmap=pylab.cm.gray)
      pylab.scatter([x for (x,y,d) in f0.kp], [y for (x,y,d) in f0.kp], label = 'f0 kp', c = 'red')
      pylab.scatter([x for (x,y,d) in f1.kp], [y for (x,y,d) in f1.kp], label = 'f1 kp', c = 'green')
      pylab.legend()
      pylab.show()

    return prox

  def plot(self, color):
    pts = dict([ (f,self.newpose(f).xform(0,0,0)) for f in self.nodes ])
    nodepts = pts.values()
    pylab.scatter([x for (x,y,z) in nodepts], [z for (x,y,z) in nodepts], c = color, label = 'after SGD')
    for (f0,f1) in self.edges:
      p0 = pts[f0]
      p1 = pts[f1]
      pylab.plot((p0[0], p1[0]), (p0[2], p1[2]), c = color)
