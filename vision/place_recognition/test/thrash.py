import rostools
rostools.update_path('place_recognition')
import rostest
import pylab, numpy
import Image

import sys
import random

sys.path.append('lib')
import place_recognition

ims = [ Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (20 * i)) for i in range(100)]

random.seed(0)

BUILD_TREE = True
if BUILD_TREE:
  vt = place_recognition.vocabularytree()
  vt.build(random.sample(ims, 50), 5, 4, False)
  vt.save("thrash.tree")
else:
  vt = place_recognition.load("thrash.tree")

for (a,q) in zip(random.sample(ims, 100), random.sample(ims, 100)):
  print a, q
  vt.add(a)
  vt.topN(q)
