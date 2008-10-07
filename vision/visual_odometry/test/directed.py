import rostools
rostools.update_path('visual_odometry')
import rostest

import sys
sys.path.append('lib')

import visual_odometry as VO
import Image
import ImageChops
import ImageDraw

import random
import time
import unittest
import math

# print VO.StereoFrame(0, "/u/konolige/vslam/data/indoor1/left-0000.ppm", "/u/konolige/vslam/data/indoor1/right-0000.ppm")

dir = "/u/konolige/vslam/data/indoor1/"
seq = [VO.StereoFrame(i, "%s/left-%04d.ppm" % (dir, i), "%s/right-%04d.ppm" % (dir, i)) for i in range(1000,1010)]
seq += [VO.StereoFrame(i, "%s/left-%04d.ppm" % (dir, 1009-i), "%s/right-%04d.ppm" % (dir, 1009-i)) for i in range(10)]

intent = range(1000,1010) + list(reversed(range(1000,1010)))
seq = [VO.StereoFrame(i, "%s/left-%04d.ppm" % (dir, f), "%s/right-%04d.ppm" % (dir, f)) for (i,f) in enumerate(intent)]

poses = VO.visual_odometry(seq)
for (f,rod,shift) in poses:
  print f, rod, shift
