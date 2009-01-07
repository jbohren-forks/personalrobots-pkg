import rostools
rostools.update_path('vslam')

import Image
from votools import TreeOptimizer3
import place_recognition
from visualodometer import VisualOdometer, from_xyz_euler
import pylab

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
    self.vo = VisualOdometer(cam)
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

  def PE(self, af0, af1):
    return self.vo.proximity(af0, af1)

  def plot(self, color):
    pts = dict([ (f,self.newpose(f).xform(0,0,0)) for f in self.nodes ])
    nodepts = pts.values()
    pylab.scatter([x for (x,y,z) in nodepts], [z for (x,y,z) in nodepts], c = color, label = 'after SGD')
    for (f0,f1) in self.edges:
      p0 = pts[f0]
      p1 = pts[f1]
      pylab.plot((p0[0], p1[0]), (p0[2], p1[2]), c = color)
