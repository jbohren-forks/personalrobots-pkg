import rostools
rostools.update_path('place_recognition')
import rostest
import pylab, numpy
import Image
import starfeature

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
if 1:
  print "loading..."
  vt = place_recognition.load("foo")

  print "adding..."
  for i in ims:
    vt.add(i)
    
  def kp_d(frame):
    sd = starfeature.star_detector(frame.size[0], frame.size[1], 5, 10.0, 10.0)
    kp = [ (x,y) for (x,y,s,r) in sd.detect(frame.rawdata) ]
    cl = calonder.classifier()
    cl.read('/u/prdata/calonder_trees/current.rtc')

    descriptors = []
    im = Image.fromstring("L", frame.size, frame.rawdata)
    matcher = calonder.BruteForceMatcher(176)
    for (x,y) in kp:
      patch = im.crop((x-16,y-16,x+16,y+16))
      sig = cl.getSignature(patch.tostring(), patch.size[0], patch.size[1])
      descriptors.append(sig)
    return descriptors

  #M = vt.query(ims)
  print "Do query"
  M = [ vt.topN(i, kp_d(i), 10) for i in ims[:100] ]

  pylab.pcolor(numpy.array(M))
  pylab.colorbar()
  pylab.show()
