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

import sys
import time
import getopt

from math import *
import matplotlib
matplotlib.use('cairo.pdf')
import networkx as nx

from stereo_utils.stereo import DenseStereoFrame, SparseStereoFrame
from visual_odometry.visualodometer import VisualOdometer, Pose, from_xyz_euler
from stereo_utils.descriptor_schemes import DescriptorSchemeCalonder, DescriptorSchemeSAD
from stereo_utils.feature_detectors import FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
from skeleton import Skeleton
import sensor_msgs.msg
import robot_msgs.msg
from tf.listener import TransformListener

from pytoro import TreeOptimizer3

from stereo_utils import camera
import rospy
import Image

import time
import math

import vslam.msg

class RoadmapServer:
    def __init__(self, args):
        rospy.init_node('roadmap_server')
        self.optimization_distance = int(args[1])
        self.tf = TransformListener()
        stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))
        self.skel = Skeleton(stereo_cam)
        # self.skel.node_vdist = 1
        if False:
          self.skel.load(args[1])
          self.skel.optimize()
          self.startframe = 100000
        else:
          self.startframe = 0

        self.frame_timestamps = {}
        self.vo = None

        self.pub = rospy.Publisher("/roadmap", vslam.msg.Roadmap)

        time.sleep(1)
        #self.send_map(rospy.time(0))
        self.wheel_odom_edges = set()

        rospy.Subscriber('/wide_stereo/raw_stereo', sensor_msgs.msg.RawStereo, self.handle_raw_stereo, queue_size=2, buff_size=7000000)

    def send_map(self, stamp):

        if len(self.skel.nodes) < 4:
          return

        TG = nx.MultiDiGraph()
        for i in self.skel.nodes:
            TG.add_node(i)
        for (a,b,p,c) in self.skel.every_edge + list(self.wheel_odom_edges):
            TG.add_edge(a, b, (p, c))
        here = max(self.skel.nodes)

        # TG is the total graph, G is the local subgraph

        uTG = TG.to_undirected()
        print "uTG", uTG.nodes(), uTG.edges()
        close = set([here])
        for i in range(self.optimization_distance):
            close |= set(nx.node_boundary(uTG, close))
        print "close", close

        G = TG.subgraph(close)
        uG = uTG.subgraph(close)
        pg = TreeOptimizer3()

        def mk_covar(xyz, rp, yaw):
            return (1.0 / math.sqrt(xyz),1.0 / math.sqrt(xyz), 1.0 / math.sqrt(xyz), 1.0 / math.sqrt(rp), 1.0 / math.sqrt(rp), 1.0 / math.sqrt(yaw))
        weak = mk_covar(9e10,3,3)
        strong = mk_covar(0.0001, 0.000002, 0.00002)

        pg.initializeOnlineOptimization()
        def pgadd(p, n, data):
            relpose, strength = data
            if strength == 0.0:
                cov = weak
            else:
                cov = strong
            pg.addIncrementalEdge(p, n, relpose.xform(0,0,0), relpose.euler(), cov)

        Gedges = G.edges(data = True)
        revmap = {}
        map = {}
        for n in nx.dfs_preorder(uG, here):
            revmap[len(map)] = n
            map[n] = len(map)
            priors = [p for p in uG.neighbors(n) if p in map]
            if priors == []:
                print "START NODE", n, "as", map[n]
            else:
                print "NEXT NODE", n
                p = priors[0]
                if not G.has_edge(p, n):
                  n,p = p,n
                data = G.get_edge_data(p, n)[0]
                Gedges.remove((p, n, data))
                pgadd(map[p], map[n], data)
        for (p,n,d) in Gedges:
          pgadd(map[p], map[n], d)
        pg.initializeOnlineIterations()
        start_error = pg.error()
        for i in range(10):
            pg.iterate()
        print "Error from", start_error, "to", pg.error()

        pg.recomputeAllTransformations()
        print self.tf.getFrameStrings()
        target_frame = "base_link"
        t = self.tf.getLatestCommonTime("double_stereo_wide_optical_frame", target_frame)
        trans,rot = self.tf.lookupTransform(target_frame, "double_stereo_wide_optical_frame", t)
        xp = Pose()
        xp.fromlist(self.tf.fromTranslationRotation(trans,rot))

        roadmap_nodes = dict([ (n, (30,0,0)) for n in TG.nodes() ])
        nl = sorted(roadmap_nodes.keys())
        print "nl", nl
        for i in sorted(map.values()):
            xyz,euler = pg.vertex(i)
            p = from_xyz_euler(xyz, euler)
            pose_in_target = xp * p
            x,y,z = pose_in_target.xform(0,0,0)
            euler = pose_in_target.euler()
            print x,y,z, euler
            roadmap_nodes[revmap[i]] = (x, y, euler[2])

        roadmap_edges = []
        for (p,n) in set(TG.edges()):
            print p,n
            best_length = max([ (confidence, pose.distance()) for (pose, confidence) in TG.get_edge_data(p, n).values()])[1]
            roadmap_edges.append((p, n, best_length))

        msg = vslam.msg.Roadmap()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.nodes = [vslam.msg.Node(*roadmap_nodes[n]) for n in nl]
        print "(p,n)", [ (p,n) for (p,n,l) in roadmap_edges ]
        msg.edges = [vslam.msg.Edge(nl.index(p), nl.index(n), l) for (p, n, l) in roadmap_edges]
        msg.localization = nl.index(here)
        self.pub.publish(msg)

        if 1:
            import matplotlib.pyplot as plt
            fig = plt.figure(figsize = (12, 6))
            plt.subplot(121)
            pos = nx.spring_layout(TG, iterations=1000)
            nx.draw(TG, pos,node_color='#A0CBE2')
            nx.draw_networkx_edges(G, pos, with_labels=False, edge_color='r', alpha=0.5, width=25.0, arrows = False)
            plt.subplot(122)
            nx.draw(G, pos = dict([(n, roadmap_nodes[n][:2]) for n in G.nodes()]))
            #plt.show()
            plt.savefig("/tmp/map_%d.png" % here)
 
    def handle_raw_stereo(self, msg):
        size = (msg.left_info.width, msg.left_info.height)
        if self.vo == None:
          cam = camera.StereoCamera(msg.right_info)
          self.fd = FeatureDetectorFast(300)
          self.ds = DescriptorSchemeCalonder()
          self.vo = VisualOdometer(cam, scavenge = False,
                              inlier_error_threshold = 3.0, sba = None,
                              inlier_thresh = 100,
                              position_keypoint_thresh = 0.2, angle_keypoint_thresh = 0.15)
          self.vo.num_frames = self.startframe
        pair = [Image.fromstring("L", size, i.uint8_data.data) for i in [ msg.left_image, msg.right_image ]]
        af = SparseStereoFrame(pair[0], pair[1], feature_detector = self.fd, descriptor_scheme = self.ds)
        self.vo.handle_frame(af)
        self.frame_timestamps[af.id] = msg.header.stamp
        
        if self.skel.add(self.vo.keyframe):
          print "====>", self.vo.keyframe.id, self.frame_timestamps[self.vo.keyframe.id]
          #print self.tf.getFrameStrings()
          #assert self.tf.frameExists("double_stereo_wide_optical_frame")
          #assert self.tf.frameExists("odom_combined")
          N = sorted(self.skel.nodes)
          for a,b in zip(N, N[1:]):
            if (a,b) in self.skel.edges:
              t0 = self.frame_timestamps[a]
              t1 = self.frame_timestamps[b]
              if self.tf.canTransformFull("double_stereo_wide_optical_frame", t0, "double_stereo_wide_optical_frame", t1, "odom_combined"):
                t,r = self.tf.lookupTransformFull("double_stereo_wide_optical_frame", t0, "double_stereo_wide_optical_frame", t1, "odom_combined")
                relpose = Pose()
                relpose.fromlist(self.tf.fromTranslationRotation(t,r))
                self.wheel_odom_edges.add((a, b, relpose, 1.0))
          self.send_map(msg.header.stamp)

def dist(a,b):
  xd = a[0] - b[0]
  yd = a[1] - b[1]
  return math.sqrt(xd*xd + yd*yd)

import tf

class FakeRoadmapServer:

  def __init__(self, args):
    rospy.init_node('roadmap_server')
    self.pub = rospy.Publisher("roadmap", vslam.msg.Roadmap)
    rospy.Subscriber('amcl_pose', robot_msgs.msg.PoseWithCovariance, self.handle_localizedpose)
    self.updated = None
    rospy.Subscriber('time', roslib.msg.Time, self.handle_time)
    self.nodes = []

  def handle_time(self, msg):
    if self.updated == None:
      self.updated = msg.rostime
    if (msg.rostime - self.updated) > roslib.rostime.Duration(5):
      self.updated = msg.rostime
      self.send_map(msg.header.stamp)

  def handle_localizedpose(self, msg):
    th = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
    x = msg.pose.position.x
    y = msg.pose.position.y
    print x, y, th
    n = (x, y, th)
    if self.nodes == [] or (dist(self.nodes[-1], n) > 1.0) or (abs(self.nodes[-1][2] - n[2]) > (2.0 * pi / 180)):
      self.nodes.append((x, y, th))
      self.send_map(msg.header.stamp)

  def send_map(self, stamp):
    p = vslam.msg.Roadmap()
    print "sending time", stamp
    p.header.stamp = stamp
    p.nodes = [ vslam.msg.Node(x,y,t) for (x,y,t) in self.nodes ]
    es = []
    for a in range(len(self.nodes)):
      for b in range(a):
        if dist(self.nodes[a], self.nodes[b]) < 1.5:
          es.append((a,b))
    print len(es), "edges"
    p.edges = [ vslam.msg.Edge(a,b) for (a,b) in es ]
    p.localization = len(self.nodes) - 1
    print "Stamp is ", stamp, " header stamp is ", p.header.stamp
    self.pub.publish(p)

def main(args):
  if (len(args) > 1) and (args[1] == 'stage'):
    rms = FakeRoadmapServer(args)
  else:
    rms = RoadmapServer(args)
  rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
