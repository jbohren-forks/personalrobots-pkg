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
roslib.load_manifest('vslam')
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
import pickle

from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler
from skeleton import Skeleton
from reader import reader

from math import *

import camera
import pylab, numpy
from matplotlib.patches import Ellipse

stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))
bad_vertices = set([ 35437, 35455, 37380, 40122, 40126, 40207, 40229 ])

if 0:
  skel = Skeleton(stereo_cam)
  skel.load("iros2009/mkplot_snap")
  print "skel.nodes:", len(skel.nodes)
  for e in skel.edges:
    if e[0] in bad_vertices or e[1] in bad_vertices:
      print "bad edge", e
  skel.nodes = [ id for id in skel.nodes if not id in bad_vertices ]
  print "skel.nodes:", len(skel.nodes)
  print set(skel.labelization())
  skel.optimize()
  skel.optimize()
  skel.save("/home/jamesb/ABCDEF2")
  pos,edges,_ = skel.localization()
  f = open("deletion2.pickle", "w")
  pickle.dump(pos, f)
  pickle.dump(edges, f)
  pickle.dump(skel.labelization(), f)
  f.close()

res_views = []
res_view_nb = []
res_clusters_nb = []

op = open("1", "w")
for kappa in [ 9999,7,5,4,3,2 ]:

  print "Starting kappa", kappa

  f = open("deletion2.pickle", "r")
  pos = pickle.load(f)
  edges = pickle.load(f)
  labels = pickle.load(f)
  f.close()

  existing = set([pos[i] for i in range(len(labels)) if "/bag" in labels[i]])
  late = set(pos) - existing

  edges = set(edges)

  existing = sorted([pos.index(p) for p in existing])
  late = sorted([pos.index(p) for p in late])

  def is_near(i0, i1):
    xd = pos[i0][0] - pos[i1][0]
    yd = pos[i0][1] - pos[i1][1]
    zd = pos[i0][2] - pos[i1][2]
    return sqrt(xd*xd + yd*yd + zd*zd) < 1.0

  def is_connected(i0, i1):
    return (i0,i1) in edges or (i1,i0) in edges

  ts = dict([(a,a) for a in range(len(existing))])

  log_numviews = []
  log_numclusters = []

  for id in sorted(late):
    print
    print id
    neighbors = set([ i1 for i1 in existing if is_near(id, i1) ])
    print "neighbors", neighbors

    existing.append(id)

    if len(neighbors) == 0:
      ts[id] = -1
    else:
      ts[id] = id
      while True:
        neighbors = set([ i1 for i1 in existing if is_near(id, i1) ])
        nviews = len(neighbors)
        oldest = min([ (ts[x],x) for x in neighbors ])[1]
        ne = set([e for e in edges if e[0] in neighbors and e[1] in neighbors])
        print ne
        clusters = []
        while len(neighbors) != 0:
          seed = neighbors.pop()
          cluster = set([seed])
          for i1 in neighbors:
            used_e = set([])
            for e in ne:
              if e[0] in cluster and not e[1] in cluster:
                cluster.add(e[1])
                used_e.add(e)
              if not e[0] in cluster and e[1] in cluster:
                cluster.add(e[0])
                used_e.add(e)
            if len(used_e) == 0:
              break
            ne -= used_e
          clusters.append(cluster)
          neighbors -= cluster
        print clusters

        bigclusters = [ cl for cl in clusters if len(cl) > 1 ]
        if len(bigclusters) > 0:
          # delete the oldest exemplar among these clusters
          kill =  min(sum([ [ (ts[x],x) for x in cl ] for cl in bigclusters ], []))[1]
          log_numclusters.append(len(clusters))
        else:
          kill = oldest
          log_numclusters.append(len(clusters)-1)
        kill_cluster = [ cl for cl in clusters if (kill in cl)][0]
        assert kill in kill_cluster

        print "nviews", nviews
        if nviews <= kappa:
          log_numviews.append(nviews)
          break

        print "killing", kill, "from cluster", kill_cluster
        existing.remove(kill)
        ke = set()
        for e in edges:
          if e[0] == kill or e[1] == kill:
            ke.add(e)

        # adj is nodes adjacent to the one to be killed
        adj = (set([e[0] for e in ke]) | set(e[1] for e in ke)) - set([kill])

        def n_n(n, edges):
          r = set()
          for i0,i1 in edges:
            if i0 in n:
              r.add(i1)
            if i1 in n:
              r.add(i0)
          return r

        # Do not consider adj nodes that have another connection to the cluster
        adj -= n_n(set(kill_cluster) - set([kill]), edges)

        # Compute new edges (ne)
        ne = set()
        for ei0 in adj:
          for ei1 in (adj - set([ei0])):
            if (ei0<ei1) and (not (ei0,ei1) in edges) and (not (ei1,ei0) in edges):
              ne.add((ei0,ei1))
        print "ne length", len(ne)
        if len(ne) > 90:
          print
          print "adj", adj
          print "ne", ne
          print
        edges -= ke
        edges |= ne

  print >>op
  print >>op, "kappa", kappa
  print >>op, "  final map size", len(existing)
  print >>op, "  avg views/nieghborhood", sum(log_numviews) / float(len(log_numviews))
  print >>op, "  avg clusters/neighborhood", sum(log_numclusters) / float(len(log_numclusters))
  res_views.append((len([x for x in existing if x in late])))
  res_view_nb.append((sum(log_numviews) / float(len(log_numviews))))
  res_clusters_nb.append((sum(log_numclusters) / float(len(log_numclusters))))

  vx = {}
  for l in open("iros2009/mkplot_snap.toro"):
    l = l.rstrip()
    f = l.split()
    if f[0] == 'VERTEX3':
      vx[int(f[1])] = [float(x) for x in f[2:]]
  for i in bad_vertices:
    del vx[i]
  poses = {}
  map = sorted(vx.keys())
  newtoro = open("deleted_%d.toro" % kappa, "w")
  for i in range(len(vx)):
    if i in existing:
      tup6 = vx[map[i]]
      print >>newtoro, 'VERTEX3', map[i], " ".join([str(f) for f in tup6])
      poses[i] = from_xyz_euler(tuple(tup6[:3]), tuple(tup6[3:]))
  for e in edges:
    p0 = poses[e[0]]
    p1 = poses[e[1]]
    relpose = ~p0 * p1
    xyz = relpose.xform(0,0,0)
    euler = relpose.euler()
    print >>newtoro, 'EDGE3', map[e[0]], map[e[1]], xyz[0], xyz[1], xyz[2], euler[0], euler[1], euler[2], "100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 707.107 0 0 707.107 0 223.607"
  newtoro.close()

def prty(L):
  return " & ".join(["%.1f" % x for x in L])

tab = open("table", "w")
print >>tab, ("""
!begin{tabular}{r||c|c|c|c|c|c}
 $!kappa$ & $!infty $ & 7 & 5 & 4 & 3 & 2 !!
!hline
 Views & """ + " & ".join([str(x) for x in res_views]) + """ !!
!hline
 Views/nb &  """ + prty(res_view_nb) + """ !!
!hline
 Clusters/nb &  """ + prty(res_clusters_nb) + """ !!
!hline

!end{tabular}

""").replace('!', '\\')

sys.exit(0)

fig = pylab.figure(figsize=(20,5), linewidth = 0.0)

print late
gn = pos[:1162]
ge = [(i0,i1) for (i0,i1) in edges if (i0 < 1162) and (i1 < 1162)]

timestamps = dict([(i,i) for i in range(len(gn))])
deleted = set()
def dist(n0, n1):
  xd = n0[0] - n1[0]
  yd = n0[1] - n1[1]
  return sqrt(xd*xd + yd*yd)

for i in range(4):
  s = fig.add_subplot(1,4,1+i)
  s.set_position( [ 0.25 * i, 0.0, 0.25, 1.00 ] )

  id = 1162 + i
  neighbors = set([j for j in range(len(gn)) if dist(pos[id], gn[j]) < 0.5]) - deleted
  gn += [pos[id]]
  el = Ellipse((pos[id][0],pos[id][1]), 1.0, 1.0, facecolor='#f0f0f0', zorder = 0)
  s.add_artist(el)
  el.set_clip_box(s.bbox)

  for i0,i1 in edges:
    if i0 < len(ge) and i1 < len(ge) and not i0 in deleted and not i1 in deleted:
      pylab.plot([pos[i0][0], pos[i1][0]], [pos[i0][1], pos[i1][1]], color='lightblue', zorder = 1)
  todraw = set(range(len(gn))) - deleted
  pylab.scatter([pos[i][0] for i in todraw], [pos[i][1] for i in todraw], c=[i for i in todraw], zorder = 10)

  todel = min(neighbors)
  pylab.scatter([pos[todel][0]], [pos[todel][1]], c='black', marker = 'x', zorder = 11, s = 400)
  print len(neighbors), "deleting", todel
  deleted.add(todel)

  #pylab.scatter([x for (x,y,r) in late], [y for (x,y,r) in late], c='g', zorder = 10)
  #for p in late:
  #  pylab.annotate('%d' % pos.index(p), (p[0], p[1]))

  pylab.xlim(-18,-16)
  pylab.ylim(-1.5,0.5)
  s.set_yticklabels([])
  s.set_xticklabels([])

pylab.savefig("deletion.eps")
pylab.show()
