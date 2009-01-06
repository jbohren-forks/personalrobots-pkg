import rostools
rostools.update_path('star_detector')
import rostest
import pylab, numpy
import Image

import sys
sys.path.append('lib')
import place_recognition

#ims = [ Image.open("/u/jamesb/ros2/ros-pkg/vision/visual_odometry/key_%06d.png" % (3 * i)) for i in range(1000)]
#ims = [ Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (20 * i)) for i in range(100)]

ims = [ Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (20 * i)) for i in range(10)]
if 1:
  vt = place_recognition.vocabularytree()
  vt.build(ims, 5, 4, False)
  vt.save("foo")

print "loading..."
vt = place_recognition.load("foo")

print "adding..."
for i in ims:
  vt.add(i)

#M = vt.query(ims)
M = [ vt.topN(i) for i in ims[:100] ]

pylab.pcolor(numpy.array(M))
pylab.colorbar()
pylab.show()
