import rostools
rostools.update_path('calonder_descriptor')
import rostest

import sys
sys.path.append('lib')
import calonder
import Image
import ImageChops
import ImageDraw

import random
import time
import math

import fast

im = Image.open("/u/konolige/vslam/data/indoor1/left-%04d.ppm" % 1000)
kp = [(x-16, y-16) for (x,y) in fast.fast(im.tostring(), im.size[0], im.size[1], 150, 40)]
print len(kp), "keypoints"

cl = calonder.classifier()
cl.setThreshold(0.0)

cl.train(im.tostring(), im.size[0], im.size[1], kp)
#cl.write('python.tree')

ma = calonder.BruteForceMatcher()

sigs = []
for (x,y) in kp:
  patch = im.crop((x,y,x+32,y+32))
  sig = cl.getSparseSignature(patch.tostring(), patch.size[0], patch.size[1])
  print [ "%.3f" % x for x in sig.dump()], sum(sig.dump())
  sigs.append(sig)
  ma.addSignature(sig, x)

print sigs

for (x,y) in kp:
  patch = im.crop((x,y,x+32,y+32))
  sig = cl.getSparseSignature(patch.tostring(), patch.size[0], patch.size[1])
  print "sig", [ "%.3f" % x for x in sig.dump()], sum(sig.dump())
  print ma.findMatch(sig)
  print 
