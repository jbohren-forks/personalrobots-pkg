import rostools
rostools.update_path('star_detector')
import rostest
import pylab, numpy
import Image

import sys
import random

sys.path.append('lib')
import place_recognition

ims = [ Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (20 * i)) for i in range(100)]

vt = place_recognition.vocabularytree()
random.seed(0)

vt.build(random.sample(ims, 50), 5, 4, False)

for (a,q) in zip(random.sample(ims, 100), random.sample(ims, 100)):
  print a, q
  vt.add(a)
  vt.topN(q)
