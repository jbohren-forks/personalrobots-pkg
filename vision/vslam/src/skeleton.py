import Image

from pytoro import TreeOptimizer3
import place_recognition
from visual_odometry.visualodometer import VisualOdometer, Pose, from_xyz_euler
from visual_odometry.pe import PoseEstimator
from stereo_utils.stereo import SparseStereoFrame
from stereo_utils.timer import Timer
from stereo_utils.descriptor_schemes import DescriptorSchemeCalonder

import numpy
import cPickle as pickle
import math
import os
import time

class minimum_frame:
  def __init__(self, id, kp, descriptors):
    self.id = id
    self.kp = kp
    self.descriptors = descriptors
    assert len(kp) == len(descriptors)

def mk_covar(xyz, rp, yaw):
  return (1.0 / math.sqrt(xyz),1.0 / math.sqrt(xyz), 1.0 / math.sqrt(xyz), 1.0 / math.sqrt(rp), 1.0 / math.sqrt(rp), 1.0 / math.sqrt(yaw))
#weak = mk_covar(0.01, 0.0002, 0.002)

weak = mk_covar(9e10,3,3)
strong = mk_covar(0.0001, 0.000002, 0.00002)

def default_termcrit(count, delta):
  return ((count > 10) or (delta < 1e-1))

class Skeleton:

  def __init__(self, cam, **kwargs):
    self.nodes = set()
    self.edges = set()
    self.weak_edges = []
    if 1:
      self.pg = TreeOptimizer3()
      self.vset = set()
    else:
      import faketoro
      self.pg = faketoro.faketoro()
    self.pg.initializeOnlineOptimization()

    self.place_ids = []
    self.cam = cam
    self.pe = PoseEstimator(*cam.params)

    self.optimize_after_addition = True
    self.ds = None
    for k,a in kwargs.items():
      if k == 'descriptor_scheme':
        self.ds = a
      elif k == 'optimize_after_addition':
        self.optimize_after_addition = a
    if self.ds == None:
      self.ds = DescriptorSchemeCalonder()

    self.vt = None
    search = [ '/u/mihelich/images/holidays/holidays.tree', '/u/jamesb/holidays.tree' ]
    for filename in search:
      if os.access(filename, os.R_OK):
        self.vt = place_recognition.load(filename, self.ds.cl)
        break
    if not self.vt:
      print "ERROR: Could not find a valid place_recognition tree in", search
      assert 0

    self.node_kp = {}
    self.node_descriptors = {}

    self.termcrit = default_termcrit
    self.pr_maximum = 15    # How many out of PR's places to consider for GCC
    self.node_vdist = 15    # how many frame to wait to put in a skeleton node
    self.adaptive = False
    self.label = "no label"
    self.node_labels = {}

    self.fills = False

    self.timer = {}
    for t in ['toro add', 'toro opt', 'place recognition', 'gcc', 'descriptors']:
      self.timer[t] = Timer()

  def save(self, basename):
    f = open(basename + ".pickle", "w")
    pickle.dump(self.nodes, f)
    pickle.dump(self.node_labels, f)
    pickle.dump(self.edges, f)
    pickle.dump(self.place_ids, f)
    pickle.dump(self.node_kp, f)
    pickle.dump(self.node_descriptors, f)
    f.close()
    self.pg.save(basename + ".toro")

  def setlabel(self, str):
    self.label = str

  def load(self, basename, load_PR = True):
    f = open(basename + ".pickle", "r")
    self.nodes = pickle.load(f)
    self.node_labels = pickle.load(f)
    self.edges = pickle.load(f)
    self.place_ids = pickle.load(f)
    if load_PR:
      self.node_kp = pickle.load(f)
      self.node_descriptors = pickle.load(f)
    self.prev_pose = None
    f.close()

    if load_PR:
      for id in self.place_ids:
        self.vt.add(None, self.node_descriptors[id])

    if 0:
      self.pg.load(basename + ".toro")
    else:
      edges = set()
      for l in open(basename + ".toro"):
        fld = l.split()
        if fld[0] == 'EDGE3':
          flt = [ float(x) for x in fld[3:] ]
          cov = [flt[i] for i in [ 6,12,17,21,24,26 ]]
          n0 = int(fld[1])
          n1 = int(fld[2])
          edges.add((n0, n1, tuple(flt[0:3]), tuple(flt[3:6]), tuple(cov)))
      reached = set([0])
      while len(edges) != 0:
        done = set()
        for e in edges:
          if e[0] in reached:
            self.pg.addIncrementalEdge(*e)
            reached.add(e[1])
            done.add(e)
          elif e[1] in reached:
            revpose = ~from_xyz_euler(e[2], e[3])
            self.pg.addIncrementalEdge(e[1], e[0], revpose.xform(0,0,0), revpose.euler(), e[4])
            reached.add(e[0])
            done.add(e)
        edges -= done
        if done == set():
          print "Graph has unreachable nodes"
          print "reached", sorted(reached)
          print "remaining edges", sorted([e[:2] for e in edges])
          sys.exit(1)
    print "Loaded", basename, ":", len(self.nodes), "nodes;", len(self.edges), "edges"

  def fill(self, filename):

    startframe = None

    if len(self.nodes) != 0:
      my_start = max(self.nodes) + 1
    else:
      my_start = 0

    from tf import transformations

    self.fills = {}
    for l in open(filename):
      f = l.split()

      if len(f) == 11:

        if not startframe:
          startframe = int(f[1])

        assert f[4] == 'pose:'
        X = float(f[5])
        Y = float(f[6])
        Z = float(f[7])
        if 0:
          roll = math.pi * float(f[8]) / 180.0 
          pitch = math.pi * float(f[9]) / 180.0
          yaw = math.pi * float(f[10]) / 180.0

          sa = math.sin(yaw);
          sb = math.sin(pitch);
          sg = math.sin(roll);

          ca = math.cos(yaw);
          cb = math.cos(pitch);
          cg = math.cos(roll);

          P = numpy.array(
            [ [ ca*cb , -sa*cg + ca*sb*sg , sa*sg + ca*sb*cg  ],
              [ sa*cb , ca*cg + sa*sb*sg  , -ca*sg + sa*sb*cg ],
              [ -sb   , cb*sg             , cb*cg             ]]).T
        else:
          euler = [ float(c) * math.pi / 180 for c in f[8:11] ]

          sa = math.sin(euler[2]);
          sb = math.sin(euler[1]);
          sg = math.sin(euler[0]);

          ca = math.cos(euler[2]);
          cb = math.cos(euler[1]);
          cg = math.cos(euler[0]);

          P = numpy.zeros((3,3))
          P[0,0] = ca*cb;
          P[0,1] = -sa*cg + ca*sb*sg;
          P[0,2] = sa*sg + ca*sb*cg;

          P[1,0] = sa*cb;
          P[1,1] = ca*cg + sa*sb*sg;
          P[1,2] = -ca*sg + sa*sb*cg;

          P[2,0] = -sb;
          P[2,1] = cb*sg;
          P[2,2] = cb*cg;

        cam90 = Pose(numpy.array([[ 1, 0, 0 ], [ 0, 0, -1 ], [ 0, 1, 0 ]]), numpy.array([0, 0, 0]))
        veh_t = Pose(P, numpy.array([X, Y, Z]))
        Tc2v = Pose(
          numpy.array([
            [0.010546  , -0.006048, 0.999926 ],
            [-0.999650 , 0.024210 , 0.010689 ],
            [-0.024273 ,-0.999689 ,-0.005791 ]]),
          numpy.array(
            [ 1.783353 , 0.265480 , -0.041039 ]))
        self.fills[my_start + int(f[1])-startframe] = (~Tc2v * veh_t) * Tc2v

    if 0:
      import pylab, sys
      po = [ self.fills[x].xform(0,0,0) for x in sorted(self.fills.keys()) ]
      pylab.scatter([x for (x,y,z) in po], [y for (x,y,z) in po], c = 'r')

      print self.fills[0].M

      po = [ self.gt(0, x).xform(0,0,0) for x in sorted(self.fills.keys()) ]
      pylab.scatter([x for (x,y,z) in po], [z for (x,y,z) in po], c = 'g')
      pylab.show()
      sys.exit(1)

  def gt(self, i0, i1):
    assert i0 <= i1
    r = ~self.fills[i0] * self.fills[i1]
    return r

    p = Pose()
    p.M[0,0] =  r.M[0,0]
    p.M[0,1] =  r.M[0,2]
    p.M[0,2] = -r.M[0,1]
    p.M[0,3] =  r.M[0,3]

    p.M[1,0] = -r.M[2,1]
    p.M[1,1] =  r.M[2,2]
    p.M[1,2] = -r.M[2,0]
    p.M[1,3] = -r.M[2,3]

    p.M[2,0] = -r.M[1,0]
    p.M[2,1] =  r.M[1,2]
    p.M[2,2] =  r.M[1,1]
    p.M[2,3] =  r.M[1,3]
    return p
    cam90 = Pose(numpy.array([[ 1, 0, 0 ], [ 0, 0, -1 ], [ 0, 1, 0 ]]), numpy.array([0, 0, 0]))
    return cam90 * r * ~cam90

  def add(self, this, connected = 1):
    """ add frame *this* to skeleton, *connected* 1 means good link to
    previous, 0 means no link to previous - use fill data, -1 means no
    link to previous force weak link.
    """
    if len(self.nodes) == 0:
      self.nodes.add(this.id)
      self.node_labels[this.id] = self.label
      r = True
    elif not(this.id in self.nodes):
      previd = max(self.nodes)

      # Ignore the node if there are less than node_vist frames since the previous node
      if (connected == 1) and (this.id - previd) < self.node_vdist:
        return False

      if (connected == 1) and self.prev_pose:
        print "Strong link from %d to %d" % (previd, this.id)
        relpose = ~self.prev_pose * this.pose
        inf = strong
        print "LINK: Strong from obs", relpose.xform(0,0,0)
      else:
        if self.fills and (connected == 0):
          print "Strong link from %d to %d" % (previd, this.id)
          relpose = self.gt(self.prev_id, this.id)
          inf = strong
          print "LINK: Strong from fill", relpose.xform(0,0,0)
        else:
          self.weak_edges.append((previd, this.id))
          print "Weak link from %d to %d" % (previd, this.id)
          relpose = Pose(numpy.identity(3), [ 0, 0, 0 ])
          inf = weak
          print "LINK: Weak"

      self.nodes.add(this.id)
      self.node_labels[this.id] = self.label
      self.edges.add( (previd, this.id) )
      self.timer['toro add'].start()
      vtop = self.pg.addIncrementalEdge(previd, this.id, relpose.xform(0,0,0), relpose.euler(), inf)
      self.vset.add(previd)
      self.vset.add(this.id)
      print self.vset
      print "ADDED VO CONSTRAINT", previd, this.id, inf
      self.timer['toro add'].stop()
      #print "added node at", this.pose.xform(0,0,0), "in graph as", self.newpose(this.id).xform(0,0,0)

      self.memoize_node_kp_d(this)

      far = [ id for id in self.place_find(this.descriptors()) if (not id in [this.id, previd])]
      self.place_ids.append(this.id)
      self.add_links(this.id, far)

      r = True
    else:
      r = False
    if r:
      self.prev_pose = this.pose
      self.prev_id = this.id
      #self.prev_frame = this

    return r

  def addConstraint(self, prev, this, relpose):
    self.edges.add((prev, this))
    self.timer['toro add'].start()
    self.pg.addIncrementalEdge(prev, this, relpose.xform(0,0,0), relpose.euler(), strong)
    self.timer['toro add'].stop()

  def trim(self):
    for e in self.weak_edges:
      print "Removing weak edge", e
      self.pg.removeEdge(e[0], e[1])
    self.edges -= set(self.weak_edges)
    self.weak_edges = []

  def ioptimize(self):
    """ incremental optimization step """
    print "incremental:"
    self.pg.initializeOnlineIterations()
    self.vset = set(self.nodes)
    for i in range(100):
      self.pg.iterate(self.vset, True)
    self.pg.recomputeAllTransformations()
    self.vset = set()
    print

  def optimize(self, iters = None, duration = None):

    self.pg.initializeOnlineIterations()

    starting_error = self.pg.error()
    if iters:
      for i in range(iters):
        self.pg.iterate()
    elif duration:
      started = time.time()
      while (time.time() - started) < duration:
        print "optimizing"
        self.pg.iterate()
    print "Error changed from", starting_error, "to", self.pg.error()
    self.pg.recomputeAllTransformations()

  def newpose(self, id):
    try:
      self.pg.recomputeAllTransformations()
      xyz,euler = self.pg.vertex(id)
      p = from_xyz_euler(xyz, euler)
    except:
      print "Failed to get vertex", id
      p = Pose()
    return p

  def place_find(self, descriptors):
    if self.vt:
      self.timer['place recognition'].start()
      scores = self.vt.topN(None, descriptors, len(self.place_ids), True)
      self.timer['place recognition'].stop()
      assert len(scores) == len(self.place_ids)+1
      return [id for (_,id) in sorted(zip(scores, self.place_ids))][:self.pr_maximum]
    else:
      return self.place_ids

  def add_links(self, this, far):
    self.timer['gcc'].start()
    coll = [ self.PE(this, f) + (f,) for f in far ]
    self.timer['gcc'].stop()
    id0 = this
    # print coll
    for inl,obs,id1 in coll:
      if 40 <= inl:
        old_error = self.pg.error()
        self.addConstraint(id0, id1, obs)
        # print "ADDED CONSTRAINT", id0, id1, "error changed from", old_error, "to", self.pg.error()

    if self.optimize_after_addition:
      t0 = self.timer['toro opt'].sum
      self.timer['toro opt'].start()
      if len(self.vset) > 4:
        self.pg.initializeOnlineIterations()
        prev_e = self.pg.error()
        self.pg.iterate(self.vset, True)
        if 1:
          count = 1
        else:
          # print
          # print "Starting OPT loop, error ", self.pg.error(), " prev error ", prev_e
          while not self.termcrit(count, prev_e - self.pg.error()):
            prev_e = self.pg.error()
            self.pg.iterate()
            count += 1
        self.vset = set()
      self.timer['toro opt'].stop()
    if 0:
      t1 = self.timer['toro opt'].sum
      td = t1 - t0
      if self.adaptive:
        if (td > 0.300):                    # too large, stretch frame additions
          self.node_vdist = 15 + (td - 0.4)*100
        else:
          self.node_vdist = 15
      #print "OPT TIMER ", 1000.0*(t1-t0), "  ITERATIONS ", count, "  FRAMES ", self.node_vdist, "  ERROR ", self.pg.error()
      #print

  def memoize_node_kp_d(self, af):
    self.timer['descriptors'].start()
    if not (af.id in self.node_kp):
      self.node_kp[af.id] = af.features()
      self.node_descriptors[af.id] = af.descriptors()
    self.timer['descriptors'].stop()

  def PE(self, af0, af1):
    pairs = [ (a,b) for (a,b,_) in self.ds.match0(self.node_kp[af1], self.node_descriptors[af1], self.node_kp[af0], self.node_descriptors[af0]) ]
    inl,R,T = self.pe.estimateC(self.cam, self.node_kp[af0], self.cam, self.node_kp[af1], pairs)
    if inl == 0:
      return (0, None)
    else:
      r33 = numpy.mat(numpy.array(R).reshape(3,3))
      return (inl, Pose(r33, numpy.array(T)))

  def drawable(self):

    pts = dict([ (id,self.newpose(id).xform(0,0,0)) for id in self.nodes ])
    nodepts = pts.values()
    edges = []
    for (f0,f1) in self.edges:
      p0 = pts[f0]
      p1 = pts[f1]
      edges.append((p0, p1))
    return (nodepts, edges)

  def digest(self):
    pts = {}
    for id in self.nodes:
      (x,y,z) = self.newpose(id).xform(0,0,0)
      xp = x
      yp = z
      pts[id] = (xp,yp)
    return (self.edges, pts, self.node_labels)

  def plot(self, color, annotate = False, theta = 0.0):

    import pylab
    pts = {}
    for id in self.nodes:
      (x,y,z) = self.newpose(id).xform(0,0,0)
      if isinstance(theta, float):
        xp = x * math.cos(theta) - z * math.sin(theta)
        yp = z * math.cos(theta) + x * math.sin(theta)
      else:
        (xp,yp) = theta(x, z)
      pts[id] = (xp,yp)

    # uniq_l is unique labels
    uniq_l = sorted(set(self.node_labels.values()))

    # labs maps labels to sets of points
    labs = dict([ (l,set([id for (id,lab) in self.node_labels.items() if lab == l])) for l in uniq_l])

    # cols maps labels to colors
    if len(uniq_l) > 1:
      cols = dict(zip(uniq_l, [ 'green', 'red', 'magenta', 'cyan', 'darkorange', 'brown', 'darkolivegreen']))
    else:
      cols = { uniq_l[0] : color }

    for lbl in uniq_l:
      nodepts = [ pts[id] for id in self.nodes if self.node_labels[id] == lbl ]
      print cols[lbl], lbl[:1]
      pylab.scatter([x for (x,y) in nodepts], [y for (x,y) in nodepts], s = 6, color = cols[lbl], label = lbl[:1])

    if annotate:
      if True:
        for (f,(x,y)) in pts.items():
          pylab.annotate('%d' % f, (float(x), float(y)))
      else:
        for f in [ min(ids) for ids in labs.values() ] + [ max(ids) for ids in labs.values() ]:
          (x,y) = pts[f]
          pylab.annotate('%d' % f, (float(x), float(y)))

    ordered = sorted(self.nodes)
    cross = 0
    for (f0,f1) in self.edges:
      # This expression is 1 if the nodes are consecutive
      # abs(ordered.index(f0) - ordered.index(f1))
      p0 = pts[f0]
      p1 = pts[f1]
      p0c = cols[self.node_labels[f0]]
      p1c = cols[self.node_labels[f1]]
      if p0c == p1c:
        color = p0c
      else:
        color = 'b:'
        cross += 1
        print "cross", (f0,f1)
      pylab.plot((p0[0], p1[0]), (p0[1], p1[1]), color, linewidth=2)
      if math.sqrt((p0[0]-p1[0])**2 + (p0[1]-p1[1])**2) > 10:
        pylab.annotate('%d' % f0, p0)
        pylab.annotate('%d' % f1, p1)
    print "There are", cross, "cross links"

  def labelization(self):
    return [ self.node_labels[id] for id in sorted(self.nodes) ]

  # returns a summary of the skeleton - intended recipient is planning.
  def localization(self):
    def planar(x, y, z):
      from scipy import optimize
      def rms(sol, args):
          a,b,c,d = sol
          x,y,z = args
          return sum((y - (a*x + b*y + c*z + d)) ** 2)
      sol = [1.0, 1.0, 1.0, 0.0]
      sol = optimize.fmin(rms, sol, args=((x,y,z),))
      return sol

    Ns = sorted(list(self.nodes))
    poses = [ self.newpose(id) for id in Ns ]
    nodepts = [ p.xform(0,0,0) for p in poses ]
    nodedirs = [ p.xform(0,0,1) for p in poses ]
    a,b,c,d = planar(numpy.array([x for (x,y,z) in nodepts]), numpy.array([y for (x,y,z) in nodepts]), numpy.array([z for (x,y,z) in nodepts]))
    mag = math.sqrt(float(a*a + b*b + c*c))
    a /= mag
    b /= mag
    c /= mag
    plane_xform = numpy.array([[ b, c, a ], [ c, a, b ]])
    pos0 = [ tuple(numpy.dot(plane_xform, numpy.array( [ [x], [y], [z] ]))) for (x,y,z) in nodepts ]
    pos1 = [ tuple(numpy.dot(plane_xform, numpy.array( [ [x], [y], [z] ]))) for (x,y,z) in nodedirs ]
    u = [ p1[0] - p0[0] for (p0,p1) in zip(pos0, pos1) ]
    v = [ p1[1] - p0[1] for (p0,p1) in zip(pos0, pos1) ]
    thetas = [ math.atan2(vi, ui) for (ui, vi) in zip(u,v) ]
    return_positions = [ tuple(numpy.dot(plane_xform, numpy.array( [ [x], [y], [z] ])).transpose()[0]) + (thetas[i],) for (i, (x,y,z)) in enumerate(nodepts) ]

    reversal = dict([(Ns[i],i) for i in range(len(Ns))])

    return_edges = [ (reversal[a], reversal[b]) for (a,b) in self.edges ]

    if len(Ns) > 0:
      return_loc = len(Ns) - 1
    else:
      return_loc = -1
    return (return_positions, return_edges, return_loc)

    localizations = []
    far = [id for id in self.place_find(qf.descriptors())]
    assert 0
    coll = [ self.vo.proximity(qf, self.my_frame(f)) + (f,) for f in far ]
    print coll
    for inl,obs,id1 in coll:
      if 40 <= inl:
        localizations.append((id1, obs))
    print localizations

  def summary(self):
    pts = dict([ (id,self.newpose(id).xform(0,0,0)) for id in self.nodes ])
    return (pts, self.edges)

  def average_time_per_frame(self):
    niter = len(self.nodes)
    return 1e3 * sum([t.sum for t in self.timer.values()]) / niter

  def summarize_timers(self):
    print "Graph has", len(self.nodes), "nodes and", len(self.edges), "edges"
    print
    niter = len(self.nodes)
    if niter != 0:
      for n,t in self.timer.items():
        print "%-20s %fms" % (n, 1e3 * t.sum / niter)
      print "%-20s %fms" % ("TOTAL", self.average_time_per_frame())

  def dump_timers(self, filename):
    f = open(filename, 'w')
    d = dict([ (nm, tm.log) for (nm, tm) in self.timer.items() ])
    pickle.dump(d, f)
    f.close()
